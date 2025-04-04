#include "precision_land_test/precision_land_test.hpp"

PrecisionLandTest::PrecisionLandTest() : Node("precision_land_test")
{
    auto critical_qos = rclcpp::QoS(10)
		.reliable()
		.durability_volatile();  // Critical commands (must not lose)

	auto vision_pose_qos = rclcpp::QoS(5)
		.reliable()
		.durability_volatile();  // Vision poses (RELIABLE but bounded)

    std::string _camera_namespace_color = "/color_camera";
    std::string _camera_namespace_bnw = "/bnw_camera";

    std::string target_pose_color_topic = _camera_namespace_color.empty() 
		? "/target_pose" 
		: _camera_namespace_color + "/target_pose";

	std::string target_pose_bnw_topic = _camera_namespace_bnw.empty()
		? "/target_pose"
		: _camera_namespace_bnw + "/target_pose";

    // auto qos = rclcpp::QoS(1)
    //     .best_effort()
    //     .durability_volatile();  // PX4 control (low latency)
    // auto qos = rclcpp::QoS(10).reliable(); // Ensure reliable message delivery
    // auto qos = rclcpp::QoS(5).reliable();

    _is_active_cam_color_sub = this->create_subscription<std_msgs::msg::Bool>(
		"/is_active_cam_color",
		critical_qos,
		std::bind(&PrecisionLandTest::is_active_cam_color_callback, this, std::placeholders::_1));

    target_pose_color_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_color_topic, vision_pose_qos,
        std::bind(&PrecisionLandTest::targetPoseColorCallback, this, std::placeholders::_1));

    target_pose_bnw_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_bnw_topic, vision_pose_qos,
        std::bind(&PrecisionLandTest::targetPoseBnwCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PrecisionLandTest Node Initialized!");
}

// void PrecisionLandTest::targetPoseColorCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//     RCLCPP_INFO(this->get_logger(), "Received Color Pose: [%.2f, %.2f, %.2f]",
//                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
// }

// void PrecisionLandTest::targetPoseBnwCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//     RCLCPP_INFO(this->get_logger(), "Received BnW Pose: [%.2f, %.2f, %.2f]",
//                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
// }

void PrecisionLandTest::targetPoseColorCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
 {
	 if (_is_active_cam_color){
        RCLCPP_INFO(this->get_logger(), "Received Color Pose: [%.2f, %.2f, %.2f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	 }
 }
 
 void PrecisionLandTest::targetPoseBnwCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
 {
	 if (!(_is_active_cam_color)){
        RCLCPP_INFO(this->get_logger(), "Received BnW Pose: [%.2f, %.2f, %.2f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	 }	
 }

 void PrecisionLandTest::is_active_cam_color_callback(const std_msgs::msg::Bool::SharedPtr msg)
 {
	 if (msg->data)
		 _is_active_cam_color = true;
	 else
		 _is_active_cam_color = false;
 }