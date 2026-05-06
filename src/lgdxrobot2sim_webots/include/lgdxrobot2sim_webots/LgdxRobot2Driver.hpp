#ifndef LGDXROBOT2DRIVER_HPP
#define LGDXROBOT2DRIVER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <string>

// AVANT
// #define CHASSIS_LX 0.082
// #define CHASSIS_LY 0.104
// #define WHEEL_RADIUS 0.0375

// APRÈS — supprimer CHASSIS_LX et CHASSIS_LY, garder rayon + ajouter entraxe
#define WHEEL_RADIUS   0.0375
#define WHEEL_BASE     0.20714  // entraxe = 0.10357 * 2, à mesurer sur robot réel

namespace LgdxRobot2 
{
class LgdxRobot2Driver : public webots_ros2_driver::PluginInterface
{
  private:
    webots_ros2_driver::WebotsNode *rosNode;

    // AVANT : WbDeviceTag wheels[4];
    // APRÈS :
    WbDeviceTag wheels[2];           // 0=gauche, 1=droite
    double wheelsVelocity[2] = {0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr softwareEmergencyStopSubscription;

    bool isCrticialStatus = false;
    
    double lastSimTime = 0;

    // AVANT : double motorLastPosition[4] = {0};
    // APRÈS :
    double motorLastPosition[2] = {0};
    double robotTransform[3] = {0}; // x, y, rotation

    // AVANT : WbDeviceTag positionSensors[4];
    // APRÈS :
    WbDeviceTag positionSensors[2];

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher;

    WbDeviceTag inertialUnit;

    void cmdVelCallback(const geometry_msgs::msg::Twist &msg);

  public:
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;
};
}

#endif