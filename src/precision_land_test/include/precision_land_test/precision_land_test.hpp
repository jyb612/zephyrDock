#ifndef PRECISION_LAND_TEST_HPP
#define PRECISION_LAND_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/bool.hpp>

class PrecisionLandTest : public rclcpp::Node
{
public:
    PrecisionLandTest();

private:
    void targetPoseColorCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void targetPoseBnwCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void is_active_cam_color_callback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_color_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_bnw_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _is_active_cam_color_sub;

    bool _is_active_cam_color = true;
    float _above_ground_altitude = {};		// AMSL: Above Mean Sea Level (Origin)
    
};

#endif // PRECISION_LAND_TEST_HPP
