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
    float _param_vel_p_gain = {};
    float _param_vel_i_gain = {};
    float _param_max_velocity = {};
    float _param_target_timeout = {};
    float _param_delta_position = {};
    float _param_delta_velocity = {};

    float _vel_x_integral {};
    float _vel_y_integral {};

	// for attach (gripper)
	float _target_hover_altitude = 0.5f; // Target hover altitude (0.5 meters above ArUco marker)
	double _z_from_aruco;  // Add this line to store the original_z
	std_msgs::msg::Float64 _altitude_msg;
	std_msgs::msg::Bool _done_msg;
	bool _isloaded = false;
	bool _is_active_cam_color = true;
	int _aruco_id = 0;
	float _target_z = 0.4;
	float _above_ground_altitude = 0.0;		// AMSL: Above Mean Sea Level (Origin)
	float _above_origin_altitude = 0.0;		// AGL: Above Ground Level (Obstacle / Terrain)
	float _descent_vel_tune = 0.0;
	float _loaded_robot_z = 0.5;
	float _loaded_land_z = 0.7;
	std::string _camera_namespace_color;
	std::string _camera_namespace_bnw;
	geometry_msgs::msg::PoseStamped::SharedPtr _target_pose_color_msg;
	geometry_msgs::msg::PoseStamped::SharedPtr _target_pose_bnw_msg;
};