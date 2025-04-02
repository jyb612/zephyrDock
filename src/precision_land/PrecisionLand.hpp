/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

 #pragma once

 #include <px4_ros2/components/mode.hpp>
 #include <px4_ros2/odometry/local_position.hpp>
 #include <px4_ros2/odometry/attitude.hpp>
 #include <px4_msgs/msg/trajectory_setpoint.hpp>
 #include <px4_msgs/msg/vehicle_land_detected.hpp>
 #include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
 
 #include <rclcpp/rclcpp.hpp>
 #include <geometry_msgs/msg/pose_stamped.hpp>
 #include <cmath>
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2/LinearMath/Matrix3x3.h>
 #include <geometry_msgs/msg/pose.hpp>
 #include <geometry_msgs/msg/quaternion.hpp>
 #include <vector>
 #include <std_msgs/msg/bool.hpp>
 #include <std_msgs/msg/int32.hpp> 
 #include <std_msgs/msg/float32.hpp>
 #include <std_msgs/msg/float64.hpp>
 #include <std_msgs/msg/bool.hpp>
 
 class PrecisionLand : public px4_ros2::ModeBase
 {
 public:
	 explicit PrecisionLand(rclcpp::Node& node);

	 void inclined_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);
	 void lidar_range_callback(const std_msgs::msg::Float32::SharedPtr msg);
	 void aruco_id_callback(const std_msgs::msg::Int32::SharedPtr msg);
	 void isLoadedCallback(const std_msgs::msg::Bool::SharedPtr msg);
	 void targetPoseColorCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	 void targetPoseBnwCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	 void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
	 void is_active_cam_color_callback(const std_msgs::msg::Bool::SharedPtr msg);
 
	 // See ModeBasep
	 void onActivate() override;
	 void onDeactivate() override;
	 void updateSetpoint(float dt_s) override;
 
 private:
	 struct ArucoTag {
		 Eigen::Vector3d position;
		 Eigen::Quaterniond orientation;
		 rclcpp::Time timestamp;
 
		 bool valid() { return timestamp.nanoseconds() > 0; };
	 };
 
	 void loadParameters();
 
	 ArucoTag getTagWorld(const ArucoTag& tag);
 
	 Eigen::Vector2f calculateVelocitySetpointXY();
	 bool checkTargetTimeout();
	 bool positionReached(const Eigen::Vector3f& target) const;
 
	 enum class State {
		 Idle,
		 Search, 	// Searches for target using a search pattern
		 Approach, 	// Positioning over landing target while maintaining altitude
		 Descend, 	// Stay over landing target while descending
		 Hover,
		 Finished
	 };
 
	 void switchToState(State state);
	 std::string stateName(State state);
 
	 // ros2
	 rclcpp::Node& _node;
	 rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _inclined_angle_sub;
	 rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _lidar_range_sub;
	 rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _aruco_id_sub;
	 rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _isloaded_sub;
	 rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_color_sub;
	 rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_bnw_sub;
	 rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _is_active_cam_color_sub;
	 rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;
 
	 rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _precision_hovering_done_pub;
 
	 // px4_ros2_cpp
	 std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	 std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	 std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
 
	 // Data
	 State _state = State::Search;
 
	 ArucoTag _tag;
	 float _approach_altitude = {};
 
	 // Land detection
	 bool _land_detected = false;
	 bool _target_lost_prev = true;
 
	 // Waypoints for Search pattern
	 std::vector<Eigen::Vector3f> _search_waypoints;
	 // Search pattern generation
	 void generateSearchWaypoints();
	 // Search pattern index
	 int _search_waypoint_index = 0;
 
	 // Parameters
	 float _param_ascent_vel = {};
	 float _param_descent_vel = {};
	 float _param_hold_vel = {};
	 float _param_vel_p_gain = {};
	 float _param_vel_i_gain = {};
	 float _param_max_velocity = {};
	 float _param_target_timeout = {};
	 float _param_delta_position = {};
	 float _param_delta_velocity = {};
 
	 float _vel_x_integral {};
	 float _vel_y_integral {};
	 
	 float _param_vel_tune = {};
	 float _param_loaded_robot_z = {};
	 float _param_loaded_land_z = {};
	 float _param_inclined_angle = {};

	 // Offsets for the color camera
	 float _param_color_cam_gripper_offset_front = {};
	 float _param_color_cam_gripper_offset_right = {};
	 float _param_color_cam_gripper_offset_down = {};
	 float _param_lidar_color_cam_offset_front = {};
	 float _param_lidar_color_cam_offset_right = {};
	 float _param_lidar_color_cam_offset_down = {};
	 float _param_color_cam_marker_offset_front = {};
	 float _param_color_cam_marker_offset_right = {};

	 // Offsets for the black-and-white (bnw) camera
	 float _param_bnw_cam_gripper_offset_front = {};
	 float _param_bnw_cam_gripper_offset_right = {};
	 float _param_bnw_cam_gripper_offset_down = {};
	 float _param_lidar_bnw_cam_offset_front = {};
	 float _param_lidar_bnw_cam_offset_right = {};
	 float _param_lidar_bnw_cam_offset_down = {};
	 float _param_bnw_cam_marker_offset_front = {};
	 float _param_bnw_cam_marker_offset_right = {};

	 float _param_origin_height = {};
	 

	 std::string _camera_namespace_color;
	 std::string _camera_namespace_bnw;
	 std_msgs::msg::Float64 _altitude_msg;
	 std_msgs::msg::Bool _done_msg;
	 bool _isloaded = false;
	 bool _is_active_cam_color = true;
	 float _above_ground_altitude = {};		// AMSL: Above Mean Sea Level (Origin)
	 float _above_origin_altitude = {};		// AGL: Above Ground Level (Obstacle / Terrain)
	 int _aruco_id = 1;
	 float _target_z = {};
	 
 };