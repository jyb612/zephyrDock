#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>  // Standard message for aruco_id
#include <std_msgs/msg/float64.hpp>  // Standard message for marker_size
#include <std_msgs/msg/bool.hpp>  // Standard message for marker_size


class ArucoTrackerNode : public rclcpp::Node
{
public:
	ArucoTrackerNode();

private:
	void loadParameters();
	
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
	void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
	void annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target);
	void aruco_id_callback(const std_msgs::msg::Int32::SharedPtr msg);
	void marker_size_callback(const std_msgs::msg::Float64::SharedPtr msg);


	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;

	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _aruco_id_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _marker_size_sub;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_pub;
	
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _isloaded_pub;

	std::unique_ptr<cv::aruco::ArucoDetector> _detector;
	cv::Mat _camera_matrix;
	cv::Mat _dist_coeffs;

	int _param_aruco_id {};
	int _param_dictionary {};
	double _param_marker_size {};
	std::string _camera_namespace;
	// bool ascend_indicator = false;
	std_msgs::msg::Bool _isloaded;
};

