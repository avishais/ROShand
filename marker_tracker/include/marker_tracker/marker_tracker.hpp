#include <ros/ros.h>
#include "opencv2/aruco.hpp"
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include "marker_tracker/ImageSpacePoseSrv.h"
class MarkerTracker{
	bool new_image_received_ = false, show_image_ = false;
	double marker_size_;
	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;
	cv::Mat image_received_;
	std::vector<cv::Vec3d> rvecs_, tvecs_;
	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	std::vector<int> marker_ids_;
	std::vector<std::vector<cv::Point2f>> marker_corners_, rejected_candidates_;
	image_transport::ImageTransport * image_transport_;
	image_transport::Subscriber sub_image_;
	image_transport::Publisher pub_image_;
	ros::NodeHandle node_handle_;
	ros::Publisher pub_image_space_pose_;
	ros::ServiceServer srvsrvr_image_space_pose_;
public:
	MarkerTracker();
	void detectMarkers(cv::Mat);
	void estimatePose();
	void estimateMarkerImagePoses();
	void drawMarkers(cv::Mat& image);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void getImage(cv::Mat& img_in);
	void sendImage(const cv::Mat& img_out);
	void drawAxis(cv::Mat& image);
	void spin();
	bool callbackImageSpacePoseSrv(marker_tracker::ImageSpacePoseSrv::Request& req, marker_tracker::ImageSpacePoseSrv::Response& res);
	void publishMarkerPoseImageSpace();
	void drawMarkerImagePoses(cv::Mat& image);
	void readParametersFromServer();
//	void estimatePoseSingleMarker(const std::vector<cv::Point2f>& marker_corners_, std::vector<cv::Vec3d>& rvecs_, std::vector<cv::Vec3d>& tvecs_);
};
