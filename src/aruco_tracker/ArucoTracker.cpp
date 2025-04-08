#include "ArucoTracker.hpp"
#include <sstream>

ArucoTrackerNode::ArucoTrackerNode()
	: Node("aruco_tracker_node")
{
	loadParameters();

	// TODO: params to adjust detector params
	// See: https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
	auto detectorParams = cv::aruco::DetectorParameters();

	// See: https://docs.opencv.org/4.x/d1/d21/aruco__dictionary_8hpp.html
	auto dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);

	_detector = std::make_unique<cv::aruco::ArucoDetector>(dictionary, detectorParams);

	// Define all QoS profiles at the top for maintainability
	auto critical_qos = rclcpp::QoS(10).reliable();  // Critical boolean commands Camera images (RELIABLE for vision tasks)
	auto vision_pose_qos = rclcpp::QoS(5).reliable();  // For target_pose pub
	
	// Topic names (same as your original)
	std::string image_topic = _camera_namespace.empty() ? "/camera" : _camera_namespace + "/image_raw";
	std::string camera_info_topic = _camera_namespace.empty() ? "/camera_info" : _camera_namespace + "/camera_info";
	std::string target_pose_topic = _camera_namespace.empty() ? "/target_pose" : _camera_namespace + "/target_pose";
	std::string image_proc_topic = _camera_namespace.empty() ? "/image_proc" : _camera_namespace + "/image_proc";
	std::string aruco_detected_topic = _camera_namespace.empty() ? "/aruco_detected" : _camera_namespace + "/aruco_detected";

	// Subscribers (with matched QoS)
	_image_sub = create_subscription<sensor_msgs::msg::Image>(
		image_topic, critical_qos,  // Changed to critical_qos (RELIABLE)
		std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

	_camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
		camera_info_topic, critical_qos,  // Camera info same as images
		std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));

	_aruco_id_sub = create_subscription<std_msgs::msg::Int32>(
		"/aruco_id", critical_qos,
		std::bind(&ArucoTrackerNode::aruco_id_callback, this, std::placeholders::_1));

	_marker_size_sub = create_subscription<std_msgs::msg::Float32>(
		"/marker_size", critical_qos,
		std::bind(&ArucoTrackerNode::marker_size_callback, this, std::placeholders::_1));

	// Publishers (with matched QoS)
	_image_pub = create_publisher<sensor_msgs::msg::Image>(image_proc_topic, critical_qos);
	_target_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(target_pose_topic, vision_pose_qos);  // LIDAR-like data
	_isloaded_pub = create_publisher<std_msgs::msg::Bool>("/isloaded", critical_qos);
	_aruco_detected_pub = create_publisher<std_msgs::msg::Bool>(aruco_detected_topic, critical_qos);
	
	// auto qos = rclcpp::QoS(1).best_effort();
	// auto lidar_qos = rclcpp::QoS(10).best_effort();  // Keep last 10 messages, Best Effort
	// auto critical_qos = rclcpp::QoS(10).reliable();  // Keep last 10 messages, Reliable


	
	// std::string image_topic = _camera_namespace.empty() 
    //         ? "/camera" 
    //         : _camera_namespace + "/image_raw";
        
	// std::string camera_info_topic = _camera_namespace.empty()
	// 	? "/camera_info"
	// 	: _camera_namespace + "/camera_info";
	
	// std::string target_pose_topic = _camera_namespace.empty()
	// ? "/target_pose"
	// : _camera_namespace + "/target_pose";

	// std::string image_proc_topic = _camera_namespace.empty()
	// ? "/image_proc"
	// : _camera_namespace + "/image_proc";

	// std::string aruco_detected_topic = _camera_namespace.empty()
	// ? "/aruco_detected"
	// : _camera_namespace + "/aruco_detected";

	// _image_sub = create_subscription<sensor_msgs::msg::Image>(
    //         image_topic, critical_qos, 
    //         std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1)
    //     );

	// _camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
	// 	camera_info_topic, qos, 
	// 	std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1)
	// );

	// // _image_sub = this->create_subscription<sensor_msgs::msg::Image>(
	// // 		     "/camera", qos, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

	// // _camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
	// // 			   "/camera_info", qos, std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));
	
	
	// // New subscriptions for dynamic parameter updates
    // _aruco_id_sub = this->create_subscription<std_msgs::msg::Int32>(
    //     "/aruco_id", critical_qos, std::bind(&ArucoTrackerNode::aruco_id_callback, this, std::placeholders::_1));

    // _marker_size_sub = this->create_subscription<std_msgs::msg::Float32>(
    //     "/marker_size", critical_qos, std::bind(&ArucoTrackerNode::marker_size_callback, this, std::placeholders::_1));
	
	// // Publishers
	// _image_pub = create_publisher<sensor_msgs::msg::Image>(image_proc_topic, critical_qos);
	// _target_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(target_pose_topic, lidar_qos);
	// _isloaded_pub = create_publisher<std_msgs::msg::Bool>("/isloaded", critical_qos);
	// _aruco_detected_pub = create_publisher<std_msgs::msg::Bool>(aruco_detected_topic, critical_qos);
}

void ArucoTrackerNode::loadParameters()
{
	_camera_namespace = declare_parameter<std::string>("camera_namespace", "");
	declare_parameter<int>("aruco_id", 2);
	declare_parameter<int>("dictionary", 2); // DICT_4X4_250
	declare_parameter<double>("marker_size", 0.5);

	get_parameter("aruco_id", _param_aruco_id);
	get_parameter("dictionary", _param_dictionary);
	get_parameter("marker_size", _param_marker_size);
}

void ArucoTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try {
		_aruco_detected.data = false;
		// Convert ROS image message to OpenCV image
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		// Rotate image 180 degrees if _camera_namespace is "/bnw_camera"
		if (_camera_namespace == "/bnw_camera")
			cv::rotate(cv_ptr->image, cv_ptr->image, cv::ROTATE_180);

		// Detect markers
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		_detector->detectMarkers(cv_ptr->image, corners, ids);
		cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

		if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {

			std::vector<std::vector<cv::Point2f>> undistortedCorners;

			for (const auto& corner : corners) {
				std::vector<cv::Point2f> undistortedCorner;
				cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);
				undistortedCorners.push_back(undistortedCorner);
			}

			for (size_t i = 0; i < ids.size(); i++) {
				if (ids[i] != _param_aruco_id) {
					continue;
				}
				_aruco_detected.data = true;
				// Calculate marker size from camera intrinsics
				float half_size = _param_marker_size / 2.0f;
				std::vector<cv::Point3f> objectPoints = {
					cv::Point3f(-half_size,  half_size, 0),  // top left
					cv::Point3f(half_size,  half_size, 0),   // top right
					cv::Point3f(half_size, -half_size, 0),   // bottom right
					cv::Point3f(-half_size, -half_size, 0)   // bottom left
				};

				// Use PnP solver to estimate pose
				cv::Vec3d rvec, tvec;
				cv::solvePnP(objectPoints, undistortedCorners[i], _camera_matrix, cv::noArray(), rvec, tvec);
				// Annotate the image
				cv::drawFrameAxes(cv_ptr->image, _camera_matrix, cv::noArray(), rvec, tvec, _param_marker_size);

				// Quaternion from rotation matrix
				cv::Mat rot_mat;
				cv::Rodrigues(rvec, rot_mat);
				cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();

				// Publish target pose
				geometry_msgs::msg::PoseStamped pose_msg;
				pose_msg.header.stamp = msg->header.stamp;
				pose_msg.header.frame_id = "camera_frame";
				pose_msg.pose.position.x = tvec[0];
				pose_msg.pose.position.y = tvec[1];
				pose_msg.pose.position.z = tvec[2];
				pose_msg.pose.orientation.x = quat.x;
				pose_msg.pose.orientation.y = quat.y;
				pose_msg.pose.orientation.z = quat.z;
				pose_msg.pose.orientation.w = quat.w;
				
				if (_camera_namespace.find("bnw"))
				{
					// _isloaded.data = true;
					_isloaded.data = abs(float(tvec[2])) < 3.0
							? true
							: false;

					_isloaded_pub->publish(_isloaded);
				}

				// Two-phase land
				// if (pose_msg.pose.position.z > 5/(8/3.15))
				// 	ascend_indicator = true;
				// if (ascend_indicator == true && pose_msg.pose.position.z < 3/(8/3.15)){
				// 	ascend_indicator = false;
				// 	_param_aruco_id = 1;
				// }
				// RCLCPP_ERROR(get_logger(), "target aruco id %d", _param_aruco_id);

				_target_pose_pub->publish(pose_msg);

				// Annotate the image
				annotate_image(cv_ptr, tvec);

				// NOTE: we break here, meaning we only publish the pose of the first target we see
				break;
			}

		} else {
			RCLCPP_ERROR(get_logger(), "Missing camera calibration");
		}

		// Always publish image
		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header;
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = cv_ptr->image;
		_image_pub->publish(*out_msg.toImageMsg().get());
		_aruco_detected_pub->publish(_aruco_detected);

	} catch (const cv_bridge::Exception& e) {
		RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
	}
}

void ArucoTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	// Always update the camera matrix and distortion coefficients from the new message
	_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();   // Use clone to ensure a deep copy
	_dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();   // Use clone to ensure a deep copy

	// Log the first row of the camera matrix to verify correct values
	RCLCPP_INFO(get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
		    _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
		    _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
		    _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
	RCLCPP_INFO(get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f",
		    _camera_matrix.at<double>(0, 0), // fx
		    _camera_matrix.at<double>(1, 1), // fy
		    _camera_matrix.at<double>(0, 2), // cx
		    _camera_matrix.at<double>(1, 2)  // cy
		   );

	// Check if focal length is zero after update
	if (_camera_matrix.at<double>(0, 0) == 0) {
		RCLCPP_ERROR(get_logger(), "Focal length is zero after update!");

	} else {
		RCLCPP_INFO(get_logger(), "Updated camera intrinsics from camera_info topic.");

		RCLCPP_INFO(get_logger(), "Unsubscribing from camera info topic");
		_camera_info_sub.reset();
	}
}

void ArucoTrackerNode::aruco_id_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _param_aruco_id = msg->data;
    RCLCPP_INFO(get_logger(), "Updated Aruco ID: %d", _param_aruco_id);
}

void ArucoTrackerNode::marker_size_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    _param_marker_size = msg->data;
    RCLCPP_INFO(get_logger(), "Updated Marker Size: %f", _param_marker_size);
}

void ArucoTrackerNode::annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target)
{
	// Annotate the image with the target position and marker size
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2);
	stream << "X: "  << target[0] << " Y: " << target[1]  << " Z: " << target[2];
	std::string text_xyz = stream.str();

	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1;
	int thickness = 2;
	int baseline = 0;
	cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
	baseline += thickness;
	cv::Point textOrg((image->image.cols - textSize.width - 10), (image->image.rows - 10));
	cv::putText(image->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 8);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoTrackerNode>());
	rclcpp::shutdown();
	return 0;
}