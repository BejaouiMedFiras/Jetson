#include "LgdxRobot2Driver.hpp"

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>

// Odom
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace LgdxRobot2 {
void LgdxRobot2Driver::cmdVelCallback(const geometry_msgs::msg::Twist &msg) 
{
  if (isCrticialStatus)
  {
    wheelsVelocity[0] = 0;
    wheelsVelocity[1] = 0;
  }
  else
  {
    double x = msg.linear.x;
    // msg.linear.y ignoré — impossible physiquement
    double w = msg.angular.z;

    // Cinématique différentielle
    // AVANT (Mecanum) :
    // wheelsVelocity[0] = (1/R) * (x - y - (LX+LY)*w)  etc.
    // APRÈS (différentiel) :
    wheelsVelocity[0] = (1.0 / WHEEL_RADIUS) * (x - (WHEEL_BASE / 2.0) * w); // gauche
    wheelsVelocity[1] = (1.0 / WHEEL_RADIUS) * (x + (WHEEL_BASE / 2.0) * w); // droite
  }
}


void LgdxRobot2Driver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &) 
{
  rosNode = node;

  // AVANT : 4 roues
  // char wheelsNames[4][7] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  // APRÈS : 2 roues — noms correspondant au proto modifié
  char wheelsNames[2][13] = {"wheel_left", "wheel_right"};
  for (int i = 0; i < 2; i++) 
  {
    wheels[i] = wb_robot_get_device(wheelsNames[i]);
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0.0);
  }

  // AVANT : 4 encodeurs
  // char positionSensorsName[4][9] = {"encoder1", "encoder2", "encoder3", "encoder4"};
  // APRÈS : 2 encodeurs
  char positionSensorsName[2][16] = {"encoder_left", "encoder_right"};
  for (int i = 0; i < 2; i++) 
  {
    positionSensors[i] = wb_robot_get_device(positionSensorsName[i]);
    wb_position_sensor_enable(positionSensors[i], wb_robot_get_basic_time_step());
  }

  inertialUnit = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(inertialUnit, wb_robot_get_basic_time_step());

  // Subscriptions et publishers — inchangés
  cmdVelSubscription = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS().reliable(),
    std::bind(&LgdxRobot2Driver::cmdVelCallback, this, std::placeholders::_1)
  );
  softwareEmergencyStopSubscription = node->create_subscription<std_msgs::msg::Bool>(
    "cloud/software_emergency_stop", rclcpp::SensorDataQoS().reliable(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      isCrticialStatus = msg->data;
    });

  odomPublisher = node->create_publisher<nav_msgs::msg::Odometry>("agent/odom", rclcpp::SensorDataQoS().reliable());
  tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  jointStatePublisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS().reliable());
}

void LgdxRobot2Driver::step() 
{
  // Appliquer les vitesses aux 2 moteurs
  // AVANT : for (int i = 0; i < 4; i++)
  // APRÈS :
  for (int i = 0; i < 2; i++) 
  {
    wb_motor_set_velocity(wheels[i], wheelsVelocity[i]);
  }
  
  double currentSimTime = wb_robot_get_time();
  double timeElapsed = currentSimTime - lastSimTime;
  double motorPosition[2] = {0};
  double motorPositionChange[2] = {0};
  double motorForwardKinematic[3] = {0}; // vx, vy=0, w
  
  // AVANT : for (int i = 0; i < 4; i++)
  // APRÈS :
  for (int i = 0; i < 2; i++) 
  {
    motorPosition[i] = wb_position_sensor_get_value(positionSensors[i]);
    motorPositionChange[i] = motorPosition[i] - motorLastPosition[i];
  }

  const double *iuValue = wb_inertial_unit_get_roll_pitch_yaw(inertialUnit);
  robotTransform[2] = iuValue[2];

  // AVANT (cinématique directe Mecanum) :
  // motorForwardKinematic[0] = (wheel0 + wheel1 + wheel2 + wheel3) * R/4 / dt
  // motorForwardKinematic[1] = (-wheel0 + wheel1 + wheel2 - wheel3) * R/4 / dt
  // motorForwardKinematic[2] = (-wheel0 + wheel1 - wheel2 + wheel3) * R/(4*(LX+LY)) / dt

  // APRÈS (cinématique directe différentielle) :
  // vx = (v_gauche + v_droite) / 2
  // vy = 0 (pas de mouvement latéral)
  // w  = (v_droite - v_gauche) / WHEEL_BASE
  double vLeft  = (motorPositionChange[0] * WHEEL_RADIUS) / timeElapsed;
  double vRight = (motorPositionChange[1] * WHEEL_RADIUS) / timeElapsed;
  motorForwardKinematic[0] = (vLeft + vRight) / 2.0;   // vx
  motorForwardKinematic[1] = 0.0;                        // vy = 0
  motorForwardKinematic[2] = (vRight - vLeft) / WHEEL_BASE; // w

  robotTransform[0] += (motorForwardKinematic[0] * cos(robotTransform[2])) * timeElapsed;
  robotTransform[1] += (motorForwardKinematic[0] * sin(robotTransform[2])) * timeElapsed;
  // robotTransform[2] vient de l'IMU directement

  // Publication odométrie — inchangée
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, robotTransform[2]);
  geometry_msgs::msg::Quaternion odomQuaternion = tf2::toMsg(quaternion);
  rclcpp::Time currentTime = rosNode->get_clock()->now();
  
  geometry_msgs::msg::TransformStamped odomTf;
  odomTf.header.stamp = currentTime;
  odomTf.header.frame_id = "odom";
  odomTf.child_frame_id = "base_link";
  odomTf.transform.translation.x = robotTransform[0];
  odomTf.transform.translation.y = robotTransform[1];
  odomTf.transform.translation.z = 0.0;
  odomTf.transform.rotation = odomQuaternion;
  if(tfBroadcaster)
    tfBroadcaster->sendTransform(odomTf);
  
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = currentTime;
  odometry.header.frame_id = "odom";
  odometry.pose.pose.position.x = robotTransform[0];
  odometry.pose.pose.position.y = robotTransform[1];
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation = odomQuaternion;
  odometry.child_frame_id = "base_link";
  odometry.twist.twist.linear.x = motorForwardKinematic[0];
  odometry.twist.twist.linear.y = 0.0;  // toujours 0 en différentiel
  odometry.twist.twist.angular.z = motorForwardKinematic[2];
  if(odomPublisher)
    odomPublisher->publish(odometry);

  // AVANT : 4 joints
  // jointState.name = {"wheel1_link_joint", "wheel2_link_joint", "wheel3_link_joint", "wheel4_link_joint"};
  // APRÈS : 2 joints
  sensor_msgs::msg::JointState jointState;
  jointState.header.stamp = currentTime;
  jointState.name = {"wheel_left_joint", "wheel_right_joint"};
  jointState.position = {motorPosition[0], motorPosition[1]};
  double motorVelocity[2] = {0};
  for (int i = 0; i < 2; i++) 
  {
    motorVelocity[i] = (motorPosition[i] - motorLastPosition[i]) / timeElapsed;
  }
  jointState.velocity = {motorVelocity[0], motorVelocity[1]};
  if(jointStatePublisher)
    jointStatePublisher->publish(jointState);

  // AVANT : for (int i = 0; i < 4; i++)
  // APRÈS :
  for (int i = 0; i < 2; i++) 
  {
    motorLastPosition[i] = motorPosition[i];
  }
  lastSimTime = currentSimTime;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(LgdxRobot2::LgdxRobot2Driver, webots_ros2_driver::PluginInterface)