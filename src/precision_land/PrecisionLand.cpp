/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

 #include "PrecisionLand.hpp"

 #include <px4_ros2/components/node_with_mode.hpp>
 #include <px4_ros2/utils/geometry.hpp>
 #include <Eigen/Core>
 #include <Eigen/Geometry>
 
 static const std::string kModeName = "PrecisionLandCustom";
 static const bool kEnableDebugOutput = true;
 
 using namespace px4_ros2::literals;
 
 PrecisionLand::PrecisionLand(rclcpp::Node& node)
	 : ModeBase(node, kModeName)
	 , _node(node)
 {
	// ======================
	// QoS Profile Definitions
	// ======================
	auto px4_control_qos = rclcpp::QoS(1).best_effort();  // PX4 control (low latency)
		
	auto critical_qos = rclcpp::QoS(10).reliable();  // Critical commands (must not lose)

	auto sensor_qos = rclcpp::QoS(5).best_effort();  // Sensor data (tolerable drops)

	auto vision_pose_qos = rclcpp::QoS(5).reliable();  // Vision poses (RELIABLE but bounded)

	// ======================
	// Topic Name Definitions 
	// ======================
	std::string target_pose_color_topic = _camera_namespace_color.empty() 
		? "/target_pose" 
		: _camera_namespace_color + "/target_pose";

	std::string target_pose_bnw_topic = _camera_namespace_bnw.empty()
		? "/target_pose"
		: _camera_namespace_bnw + "/target_pose";

	// ======================
	// Subscribers
	// ======================

	// Critical Control Signals
	_inclined_angle_sub = _node.create_subscription<std_msgs::msg::Float32>(
		"/solar_panel_angle_in_rad", 
		critical_qos,
		std::bind(&PrecisionLand::inclined_angle_callback, this, std::placeholders::_1));

	_is_active_cam_color_sub = _node.create_subscription<std_msgs::msg::Bool>(
		"/is_active_cam_color",
		critical_qos,
		std::bind(&PrecisionLand::is_active_cam_color_callback, this, std::placeholders::_1));

	_aruco_id_sub = _node.create_subscription<std_msgs::msg::Int32>(
		"/aruco_id",
		critical_qos,
		std::bind(&PrecisionLand::aruco_id_callback, this, std::placeholders::_1));

	_isloaded_sub = _node.create_subscription<std_msgs::msg::Bool>(
		"/isloaded",
		critical_qos,
		std::bind(&PrecisionLand::isLoadedCallback, this, std::placeholders::_1));

	// Sensor Data
	_lidar_range_sub = _node.create_subscription<std_msgs::msg::Float32>(
		"/tfmini/range",
		sensor_qos,
		std::bind(&PrecisionLand::lidar_range_callback, this, std::placeholders::_1));

	// Vision Pose Estimates
	_target_pose_color_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
		target_pose_color_topic,
		vision_pose_qos,
		std::bind(&PrecisionLand::targetPoseColorCallback, this, std::placeholders::_1));

	_target_pose_bnw_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
		target_pose_bnw_topic,
		vision_pose_qos,
		std::bind(&PrecisionLand::targetPoseBnwCallback, this, std::placeholders::_1));

	// PX4 Status
	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
		"/fmu/out/vehicle_land_detected",
		px4_control_qos,
		std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));

	// ======================
	// Publishers
	// ======================
	_precision_hovering_done_pub = _node.create_publisher<std_msgs::msg::Bool>(
		"/precision_hovering_done",
		critical_qos);
	//  std::string target_pose_color_topic = _camera_namespace_color.empty()
	//  ? "/target_pose"
	//  : _camera_namespace_color + "/target_pose";
 
	//  std::string target_pose_bnw_topic = _camera_namespace_bnw.empty()
	//  ? "/target_pose"
	//  : _camera_namespace_bnw + "/target_pose";


	//  auto bool_qos = rclcpp::QoS(10).reliable();  // Keep last 10 messages, Reliable
	//  auto lidar_qos = rclcpp::QoS(10).best_effort();  // Keep last 10 messages, Best Effort

	 std::string aruco_detected_color_topic = _camera_namespace_color.empty()
	 ? "/aruco_detected"
	 : _camera_namespace_color + "/aruco_detected";

	 std::string aruco_detected_bnw_topic = _camera_namespace_bnw.empty()
	 ? "/aruco_detected"
	 : _camera_namespace_bnw + "/aruco_detected";
 
	//  _inclined_angle_sub = _node.create_subscription<std_msgs::msg::Float32>(
	// 	"/solar_panel_angle_in_rad", bool_qos, std::bind(&PrecisionLand::inclined_angle_callback, this, std::placeholders::_1));
 
	//  _lidar_range_sub = _node.create_subscription<std_msgs::msg::Float32>(
	// 	 "/tfmini/range", bool_qos, std::bind(&PrecisionLand::lidar_range_callback, this, std::placeholders::_1));
 
	//  _is_active_cam_color_sub = _node.create_subscription<std_msgs::msg::Bool>(
	// 	 "/is_active_cam_color", bool_qos, std::bind(&PrecisionLand::is_active_cam_color_callback, this, std::placeholders::_1));
 
	//  _aruco_id_sub = _node.create_subscription<std_msgs::msg::Int32>(
	// 	 "/aruco_id", bool_qos, std::bind(&PrecisionLand::aruco_id_callback, this, std::placeholders::_1));
 
	//  _isloaded_sub = _node.create_subscription<std_msgs::msg::Bool>("/isloaded", 
	// 	bool_qos, std::bind(&PrecisionLand::isLoadedCallback, this, std::placeholders::_1));

	 _aruco_detected_color_sub = _node.create_subscription<std_msgs::msg::Bool>(aruco_detected_color_topic, 
		critical_qos, std::bind(&PrecisionLand::arucoDetectedColorCallback, this, std::placeholders::_1));
	
	 _aruco_detected_bnw_sub = _node.create_subscription<std_msgs::msg::Bool>(aruco_detected_bnw_topic, 
		critical_qos, std::bind(&PrecisionLand::arucoDetectedBnwCallback, this, std::placeholders::_1));
 
	//  _target_pose_color_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(target_pose_color_topic,
	// 	lidar_qos, std::bind(&PrecisionLand::targetPoseColorCallback, this, std::placeholders::_1));
 
	//  _target_pose_bnw_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(target_pose_bnw_topic,
	// 	lidar_qos, std::bind(&PrecisionLand::targetPoseBnwCallback, this, std::placeholders::_1));
 
	//  _vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
	// 	lidar_qos, std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));
 
	//  _precision_hovering_done_pub = _node.create_publisher<std_msgs::msg::Bool>("/precision_hovering_done", bool_qos);

	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
 
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
 
	loadParameters();

 }
 
 void PrecisionLand::loadParameters()
 {
	 _node.declare_parameter<float>("ascent_vel", -1.0);
	 _node.declare_parameter<float>("descent_vel", 1.0);
	 _node.declare_parameter<float>("hold_vel", 1.0);
	 _node.declare_parameter<float>("vel_p_gain", 1.5);
	 _node.declare_parameter<float>("vel_i_gain", 0.0);
	 _node.declare_parameter<float>("max_velocity", 3.0);
	 _node.declare_parameter<float>("target_timeout", 3.0);
	 _node.declare_parameter<float>("delta_position", 0.25);
	 _node.declare_parameter<float>("delta_velocity", 0.25);
	 _node.declare_parameter<float>("loaded_robot_z", 0.3);
	 _node.declare_parameter<float>("loaded_land_z", 0.3);
	 _node.declare_parameter<float>("inclined_angle", 0.0);
	 _node.declare_parameter<float>("vel_tune", 0.5);
	 
	 // Declare parameters for color camera
	 _node.declare_parameter<float>("color_cam_gripper_offset_front", -0.14);
	 _node.declare_parameter<float>("color_cam_gripper_offset_right", 0.0);
	 _node.declare_parameter<float>("color_cam_gripper_offset_down", -0.15);
	 _node.declare_parameter<float>("lidar_color_cam_offset_front", 0.315);
	 _node.declare_parameter<float>("lidar_color_cam_offset_right", 0.175);
	 _node.declare_parameter<float>("lidar_color_cam_offset_down", -0.06);
	 _node.declare_parameter<float>("color_cam_marker_offset_front", 0.005);
	 _node.declare_parameter<float>("color_cam_marker_offset_right", 0.0);

	 // Declare parameters for bnw camera
	 _node.declare_parameter<float>("bnw_cam_gripper_offset_front", 0.0);
	 _node.declare_parameter<float>("bnw_cam_gripper_offset_right", 0.0);
	 _node.declare_parameter<float>("bnw_cam_gripper_offset_down", 0.0);
	 _node.declare_parameter<float>("lidar_bnw_cam_offset_front", 0.0);
	 _node.declare_parameter<float>("lidar_bnw_cam_offset_right", 0.0);
	 _node.declare_parameter<float>("lidar_bnw_cam_offset_down", 0.0);
	 _node.declare_parameter<float>("bnw_cam_marker_offset_front", 0.0);
	 _node.declare_parameter<float>("bnw_cam_marker_offset_right", 0.0);

	
	 _node.get_parameter("ascent_vel", _param_ascent_vel);
	 _node.get_parameter("descent_vel", _param_descent_vel);
	 _node.get_parameter("hold_vel", _param_hold_vel);
	 _node.get_parameter("vel_p_gain", _param_vel_p_gain);
	 _node.get_parameter("vel_i_gain", _param_vel_i_gain);
	 _node.get_parameter("max_velocity", _param_max_velocity);
	 _node.get_parameter("target_timeout", _param_target_timeout);
	 _node.get_parameter("delta_position", _param_delta_position);
	 _node.get_parameter("delta_velocity", _param_delta_velocity);

	 _node.get_parameter("loaded_robot_z", _param_loaded_robot_z);
	 _node.get_parameter("loaded_land_z", _param_loaded_land_z);
	 _node.get_parameter("inclined_angle", _param_inclined_angle);
	 _node.get_parameter("vel_tune", _param_vel_tune);
	 // Retrieve parameters for color camera
	 _node.get_parameter("color_cam_gripper_offset_front", _param_color_cam_gripper_offset_front);
	 _node.get_parameter("color_cam_gripper_offset_right", _param_color_cam_gripper_offset_right);
	 _node.get_parameter("color_cam_gripper_offset_down", _param_color_cam_gripper_offset_down);
	 _node.get_parameter("lidar_color_cam_offset_front", _param_lidar_color_cam_offset_front);
	 _node.get_parameter("lidar_color_cam_offset_right", _param_lidar_color_cam_offset_right);
	 _node.get_parameter("lidar_color_cam_offset_down", _param_lidar_color_cam_offset_down);
	 _node.get_parameter("color_cam_marker_offset_front", _param_color_cam_marker_offset_front);
	 _node.get_parameter("color_cam_marker_offset_right", _param_color_cam_marker_offset_right);

	 // Retrieve parameters for bnw camera
	 _node.get_parameter("bnw_cam_gripper_offset_front", _param_bnw_cam_gripper_offset_front);
	 _node.get_parameter("bnw_cam_gripper_offset_right", _param_bnw_cam_gripper_offset_right);
	 _node.get_parameter("bnw_cam_gripper_offset_down", _param_bnw_cam_gripper_offset_down);
	 _node.get_parameter("lidar_bnw_cam_offset_front", _param_lidar_bnw_cam_offset_front);
	 _node.get_parameter("lidar_bnw_cam_offset_right", _param_lidar_bnw_cam_offset_right);
	 _node.get_parameter("lidar_bnw_cam_offset_down", _param_lidar_bnw_cam_offset_down);
	 _node.get_parameter("bnw_cam_marker_offset_front", _param_bnw_cam_marker_offset_front);
	 _node.get_parameter("bnw_cam_marker_offset_right", _param_bnw_cam_marker_offset_right);

	 _node.get_parameter("origin_height", _param_origin_height);

 
	 RCLCPP_INFO(_node.get_logger(), "ascent_vel: %f", _param_ascent_vel);
	 RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	 RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
	//  _origin_height = _vehicle_local_position->positionNed().z();
	//  RCLCPP_INFO(_node.get_logger(), "Origin height: %f", _vehicle_local_position->positionNed().z());

 }
 
 void PrecisionLand::inclined_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
 {
	_param_inclined_angle = float(msg->data);
	RCLCPP_INFO(_node.get_logger(), "Surface inclined angle\n(rad): %.2f\n(deg): %.2f", _param_inclined_angle, _param_inclined_angle*180/M_PI);
 }

 void PrecisionLand::lidar_range_callback(const std_msgs::msg::Float32::SharedPtr msg)
 {
	 _above_ground_altitude = -float(msg->data);
 }
 
 void PrecisionLand::aruco_id_callback(const std_msgs::msg::Int32::SharedPtr msg)
 {
	 _aruco_id = msg->data;
	 RCLCPP_INFO(_node.get_logger(), "%d", int(_aruco_id));

 }
 
 void PrecisionLand::isLoadedCallback(const std_msgs::msg::Bool::SharedPtr msg)	// unused?
 {
	 // Process the received boolean message
	 if (msg->data)
		 _isloaded = true;
	 else
		 _isloaded = false;
 }

 void PrecisionLand::arucoDetectedColorCallback(const std_msgs::msg::Bool::SharedPtr msg)	// unused?
 {
	 // Process the received boolean message
	 if (msg->data){
		//  RCLCPP_INFO(_node.get_logger(), "color aruco");
		 _aruco_detected_color = true;
	 }
	 else
	 	 _aruco_detected_color = false;
 }

 void PrecisionLand::arucoDetectedBnwCallback(const std_msgs::msg::Bool::SharedPtr msg)	// unused?
 {
	 // Process the received boolean message
	 if (msg->data){
		//  RCLCPP_INFO(_node.get_logger(), "bnw aruco");
		 _aruco_detected_bnw = true;
	 }
	 else
	 	 _aruco_detected_bnw = false;
 }
 
 void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
 {
	 _land_detected = msg->landed;
 }
 
 void PrecisionLand::targetPoseColorCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
 {
	 if (_search_started && _is_active_cam_color){
		//  RCLCPP_INFO(_node.get_logger(), "color");
		 auto tag = ArucoTag {
			 .position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			 .orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			 .timestamp = _node.now(),
		 };
	 
		 // Save tag position/orientation in NED world frame
		 _tag = getTagWorld(tag);
		 _above_ground_altitude = msg->pose.position.z;
		 RCLCPP_INFO(_node.get_logger(), "color");
	 }
 }
 
 void PrecisionLand::targetPoseBnwCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
 {
	 if (_search_started && !(_is_active_cam_color)){
		//  RCLCPP_INFO(_node.get_logger(), "bnw");
		 float delta_y = -_param_bnw_cam_gripper_offset_front*pow(sin(_param_inclined_angle),2) - _param_bnw_cam_marker_offset_front;
		 
		 auto tag = ArucoTag {
			 .position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y+delta_y, msg->pose.position.z),
			 .orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			 .timestamp = _node.now(),
		 };
		 // Save tag position/orientation in NED world frame
		 _tag = getTagWorld(tag);
		 _above_ground_altitude = msg->pose.position.z;
	 }
 }
 
 void PrecisionLand::is_active_cam_color_callback(const std_msgs::msg::Bool::SharedPtr msg)
 {
	 if (msg->data){
		RCLCPP_INFO(_node.get_logger(), "color cam selected");
		_is_active_cam_color = true;

	 }
	 else{
		RCLCPP_INFO(_node.get_logger(), "bnw cam selected");
		_is_active_cam_color = false;
	 }
	 
 }
 
 PrecisionLand::ArucoTag PrecisionLand::getTagWorld(const ArucoTag& tag)
 {
	 // Convert from optical to NED
	 // Optical: X right, Y down, Z away from lens
	 // NED: X forward, Y right, Z away from viewer
	 Eigen::Matrix3d R;
	 R << 0, -1, 0,
		  1, 0, 0,
		  0, 0, 1;
	 Eigen::Quaterniond quat_NED(R);
 
	 auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	 auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());
 
	 Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	 Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * quat_NED;
	 Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
	 Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;
 
	 ArucoTag world_tag = {
		 .position = tag_world_transform.translation(),
		 .orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
		 .timestamp = tag.timestamp,
	 };
 
	 return world_tag;
 }
 
 void PrecisionLand::onActivate()
 {
	 generateSearchWaypoints();
	 _search_started = true;
	 switchToState(State::Search);
 }
 
 void PrecisionLand::onDeactivate()
 {
	 // No-op
 }
 
 void PrecisionLand::updateSetpoint(float dt_s)
 {
	 bool target_lost = checkTargetTimeout();
 
	 if (target_lost && !_target_lost_prev) {
		 RCLCPP_INFO(_node.get_logger(), "Target lost: State %s", stateName(_state).c_str());
	 } else if (!target_lost && _target_lost_prev) {
		 RCLCPP_INFO(_node.get_logger(), "Target acquired");
	 }
 
	 _target_lost_prev = target_lost;
 
	 // State machine
	 switch (_state) {
	 case State::Idle: {
		 // No-op -- just spin
		 break;
	 }
	 
	 case State::Search: {
		//  if (!std::isnan(_tag.position.x())) {
		// 	_approach_altitude = _vehicle_local_position->positionNed().z();
		// 	switchToState(State::Approach);
		// 	break;
		//  }
		 if (_aruco_detected_color || _aruco_detected_bnw) {
			 _approach_altitude = _vehicle_local_position->positionNed().z();
			 RCLCPP_INFO(_node.get_logger(), "%.2f", float(_approach_altitude));
			 switchToState(State::Approach);
			 if (_tag.valid())
			 	RCLCPP_INFO(_node.get_logger(), "valid");
			 else
			 	RCLCPP_INFO(_node.get_logger(), "not valid");


			 break;
		 }
 
		 auto waypoint_position = _search_waypoints[_search_waypoint_index];
 
		 _trajectory_setpoint->updatePosition(waypoint_position);
 
		 if (positionReached(waypoint_position)) {
			 _search_waypoint_index++;
			 
			 // If we have searched all waypoints, start over
			 if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				 _search_waypoint_index = 0;
			 }
		 }
		 break;
	 }
 
	 case State::Approach: {
 
		 if (target_lost) {
			 RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			 ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			 switchToState(State::Idle);
			 return;
		 }
 
		 // Approach using position setpoints
		 auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(), _approach_altitude);
 
		 _trajectory_setpoint->updatePosition(target_position);
 
		 if (positionReached(target_position)) {
			 switchToState(State::Descend);
		 }
 
		 break;
	 }
	 case State::Descend: {
		// RCLCPP_INFO(_node.get_logger(), "TEST");
		 if (target_lost) {
			 RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			 ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			 switchToState(State::Idle);
			 return;
		 }
		 Eigen::Vector2f vel = calculateVelocitySetpointXY();
		//  _above_ground_altitude = -(_vehicle_local_position->positionNed().z() - _tag.position.z());	// SIM
		//  _above_ground_altitude = tag_position.z()
		 // _above_ground_altitude = _above_ground_altitude + _tag.position.z();			// ACTUAL
		//  RCLCPP_INFO(_node.get_logger(), "Above Ground Height: %f", _above_ground_altitude);
		 if (_aruco_id == 3){	// loaded	
			 float delta_z = _param_bnw_cam_gripper_offset_front/2*sin(2*_param_inclined_angle);
			 _target_z = _param_loaded_robot_z + abs(delta_z);
		 }
		 else{
			 if (_aruco_id == 1){
				_target_z = _param_loaded_land_z;
				// RCLCPP_INFO(_node.get_logger(), "(%.2f, %.2f, %.2f)", _target_z, _param_loaded_land_z, _above_ground_altitude);
				if (_is_active_cam_color)
					_param_inclined_angle = 0.0;
			 }
		 }
			 
		 
		 if (_aruco_id == 0){
			 Eigen::Vector2f vel = calculateVelocitySetpointXY();
			 _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
			 _param_inclined_angle = 0.0;
			 if (_land_detected) {
				 switchToState(State::Finished);
			 }
		 }
		 else{
			//  RCLCPP_INFO(_node.get_logger(), "(%.2f, %.2f)", _target_z, _above_ground_altitude);
			 
			 // if (_above_ground_altitude <= _target_z + 0.05f && _above_ground_altitude >= _target_z - 0.05f)
			 if(abs(_above_ground_altitude - _target_z) <= 0.05f){
				 _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), 0), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
				// RCLCPP_INFO(_node.get_logger(), "Higher much");
				  RCLCPP_INFO(_node.get_logger(), "Reached target altitude at %.2f meters. Hovering now.", _above_ground_altitude);
		 
				 // Transition to Hover state after descent
				 switchToState(State::Hover);
			 }
			 // If LiDAR altitude is above the target, continue descending
			 else if (_above_ground_altitude > _target_z + 0.50f) {
				 _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
				//  RCLCPP_INFO(_node.get_logger(), "Higher");
			}
			 else if (_above_ground_altitude > _target_z + 0.05f) {
				 _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel*_param_vel_tune), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
				//  RCLCPP_INFO(_node.get_logger(), "Higher little");
			}
			 // If LiDAR altitude is below the target, ascend
			 else if (_above_ground_altitude < _target_z - 0.05f) {
				 _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_ascent_vel*_param_vel_tune), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
				//  RCLCPP_INFO(_node.get_logger(), "Lower");
			}
		 }
		 break;
	 }
 
	 case State::Hover: {
		 if (target_lost) {
			 RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			 ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			 switchToState(State::Idle);
			 return;
		 }
 
		 Eigen::Vector2f vel = calculateVelocitySetpointXY();
		//  _above_ground_altitude = -(_vehicle_local_position->positionNed().z() - _tag.position.z());		// SIM
		//  _above_ground_altitude = tag_position.z()
		 // _above_ground_altitude = _above_ground_altitude + _tag.position.z();		// ACTUAL
		//  RCLCPP_INFO(_node.get_logger(), "Above Ground Height: %f", _above_ground_altitude);
		 if (_aruco_id == 3){	// loaded
			 float delta_z = _param_bnw_cam_gripper_offset_front/2*sin(2*_param_inclined_angle);
			 _target_z = _param_loaded_robot_z + abs(delta_z);
		 }
		 else{
			 if (_aruco_id == 1){
				_target_z = _param_loaded_land_z;
				if (_is_active_cam_color)
					_param_inclined_angle = 0.0;
			 }
		 }
 
		//  RCLCPP_INFO(_node.get_logger(), "(%.2f, %.2f, %.2f)", _vehicle_local_position->positionNed().z(), _tag.position.z(), _above_ground_altitude);
		 // Get LiDAR reading for hover control
		//  RCLCPP_INFO(_node.get_logger(), "Hovering at LiDAR altitude: %.2f meters", _above_ground_altitude);
	 
		 // If LiDAR reading is within tolerance of 5 cm, maintain hover
		 if (_above_ground_altitude > _target_z + 1.00f) {
			 // If too high, descend slightly
			 _done_msg.data = false;
			 _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
			//  RCLCPP_INFO(_node.get_logger(), "Higher much");
		}
		 else if (_above_ground_altitude > _target_z + 0.02f) {
			 // If too low, ascend slightly
			 _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel*_param_vel_tune), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
			//  RCLCPP_INFO(_node.get_logger(), "Higher");
		}
		 else if (_above_ground_altitude < _target_z - 0.2f) {
			 // If too low, ascend slightly
			 _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_ascent_vel), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
			//  RCLCPP_INFO(_node.get_logger(), "Lower much");
		}
		 else if (_above_ground_altitude < _target_z - 0.02f) {
			 // If too low, ascend slightly
			 _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_ascent_vel*_param_vel_tune), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
			//  RCLCPP_INFO(_node.get_logger(), "Lower little");
		}
		 else {
			 // Maintain hover within ±5 cm of target altitude
			 _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_hold_vel), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
			//  RCLCPP_INFO(_node.get_logger(), "Ngam");
			if (!_done_msg.data)
				RCLCPP_INFO(_node.get_logger(), "Custom mode completed. Published true to /custom_mode_done.");
			 _done_msg.data = true;
			 _precision_hovering_done_pub->publish(_done_msg);
			 
		 }
		 break;
	 }
 
	 case State::Finished: {
		 ModeBase::completed(px4_ros2::Result::Success);
		 break;
	 }
	 } // end switch/case
 }
 
 Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY()
 {
	 float p_gain = _param_vel_p_gain;
	 float i_gain = _param_vel_i_gain;
 
	 // P component
	 float delta_pos_x = _vehicle_local_position->positionNed().x() - _tag.position.x();
	 float delta_pos_y = _vehicle_local_position->positionNed().y() - _tag.position.y();
 
	 // I component
	 _vel_x_integral += delta_pos_x;
	 _vel_y_integral += delta_pos_y;
	 float max_integral = _param_max_velocity;
	 _vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
	 _vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);
 
	 float Xp = delta_pos_x * p_gain;
	 float Xi = _vel_x_integral * i_gain;
	 float Yp = delta_pos_y * p_gain;
	 float Yi = _vel_y_integral * i_gain;
 
	 // Sum P and I gains
	 float vx = -1.f * (Xp + Xi);
	 float vy = -1.f * (Yp + Yi);
 
	 // 0.1m/s min vel and 3m/s max vel
	 vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
	 vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);
 
	 return Eigen::Vector2f(vx, vy);
 }
 
 bool PrecisionLand::checkTargetTimeout()
 {
	 if (!_tag.valid()) {
		 return true;
	 }
 
	 if (_node.now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
		 return true;
	 }
 
	 return false;
 }
 
 void PrecisionLand::generateSearchWaypoints()
 {
	 // Generate spiral search waypoints
	 // The search waypoints are generated in the NED frame
	 // Parameters for the search pattern
	 double start_x = _vehicle_local_position->positionNed().x();
	 double start_y = _vehicle_local_position->positionNed().y();
	 double current_z = _vehicle_local_position->positionNed().z();
	 double step_size = 0.5; // Adjust step size for spacing between points
	 std::vector<Eigen::Vector3f> waypoints;

	 // 9 search points in sequence
	 waypoints.push_back(Eigen::Vector3f(start_x, start_y, current_z)); // P0: Current position
	 waypoints.push_back(Eigen::Vector3f(start_x + step_size, start_y, current_z)); // P1: Right
	 waypoints.push_back(Eigen::Vector3f(start_x + step_size, start_y + step_size, current_z)); // P2: Front-right
	 waypoints.push_back(Eigen::Vector3f(start_x + step_size, start_y - step_size, current_z)); // P3: Right-behind
	 waypoints.push_back(Eigen::Vector3f(start_x, start_y - step_size, current_z)); // P4: Behind
	 waypoints.push_back(Eigen::Vector3f(start_x - step_size, start_y - step_size, current_z)); // P5: Left-behind
	 waypoints.push_back(Eigen::Vector3f(start_x - step_size, start_y, current_z)); // P6: Left
	 waypoints.push_back(Eigen::Vector3f(start_x - step_size, start_y + step_size, current_z)); // P7: Front-left
	 waypoints.push_back(Eigen::Vector3f(start_x, start_y + step_size, current_z)); // P8: Front

	 _search_waypoints = waypoints;
 }
 
 bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
 {
	 auto position = _vehicle_local_position->positionNed();
	 auto velocity = _vehicle_local_position->velocityNed();
 
	 const auto delta_pos = target - position;
	 // NOTE: this does NOT handle a moving target!
	 return (delta_pos.norm() < _param_delta_position) && (velocity.norm() < _param_delta_velocity);
 }
 
 std::string PrecisionLand::stateName(State state)
 {
	 switch (state) {
	 case State::Idle:
		 return "Idle";
 
	 case State::Search:
		 return "Search";
 
	 case State::Approach:
		 return "Approach";
 
	 case State::Descend:
		 return "Descend";
 
	 case State::Hover:
		 return "Hovering";
 
	 case State::Finished:
		 return "Finished";
 
	 default:
		 return "Unknown";
	 }
 }
 
 void PrecisionLand::switchToState(State state)
 {
	 RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	 _state = state;
 }
 
 int main(int argc, char* argv[])
 {
	 rclcpp::init(argc, argv);
	 rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	 rclcpp::shutdown();
	 return 0;
 }