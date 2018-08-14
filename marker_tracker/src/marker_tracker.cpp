#include "marker_tracker/marker_tracker.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "marker_tracker/ImageSpacePoseMsg.h"
MarkerTracker::MarkerTracker(){
	readParametersFromServer();
	marker_size_ = 0.0075;
	float cam_mat_data[9] = {824.127282, 0, 512, 0, 824.127282, 288, 0, 0, 1};
	camera_matrix_ = cv::Mat(3,3,CV_32F,cam_mat_data);
	float dist_data[5] {-0.073267,0.27436,0,0,-0.537225};
	dist_coeffs_ = cv::Mat(5,1,CV_32F,dist_data);
	dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	image_transport_ = new image_transport::ImageTransport(node_handle_);
	sub_image_ = image_transport_->subscribe("/camera/image_raw",1,&MarkerTracker::imageCallback, this);
	pub_image_ = image_transport_->advertise("camera/image_out", 1);
	pub_image_space_pose_ = node_handle_.advertise<marker_tracker::ImageSpacePoseMsg>("/marker_tracker/image_space_pose_msg",1);
	srvsrvr_image_space_pose_ = node_handle_.advertiseService("/marker_tracker/image_space_pose_srv",&MarkerTracker::callbackImageSpacePoseSrv,this);
}
void MarkerTracker::readParametersFromServer(){
	if(!node_handle_.getParam("/marker_tracker/show_image",show_image_))
		ROS_WARN("[marker_tracker] The parameter /marker_tracker/show_image is not set. Not showing the image by default.");
}
bool MarkerTracker::callbackImageSpacePoseSrv(marker_tracker::ImageSpacePoseSrv::Request& req, marker_tracker::ImageSpacePoseSrv::Response& res){
	res.ids = marker_ids_;
	for (size_t i = 0; i < marker_ids_.size(); i++){
		res.posx.push_back(tvecs_[i](0));
		res.posy.push_back(tvecs_[i](1));
		res.angles.push_back(rvecs_[i](2));
	}
	return true;
}
void MarkerTracker::detectMarkers(cv::Mat image_in){
	marker_corners_.clear();
	marker_ids_.clear();
	cv::aruco::detectMarkers(image_in, dictionary_, marker_corners_, marker_ids_);
}
void MarkerTracker::drawMarkers(cv::Mat& image){
	cv::aruco::drawDetectedMarkers(image, marker_corners_, marker_ids_);
}
void MarkerTracker::publishMarkerPoseImageSpace(){
	marker_tracker::ImageSpacePoseMsg msg;
	msg.ids = marker_ids_;
	for (size_t i = 0; i < marker_ids_.size(); i++){
		msg.posx.push_back(tvecs_[i].val[0]);
		msg.posy.push_back(tvecs_[i].val[1]);
		msg.angles.push_back(rvecs_[i].val[2]);
	}
	pub_image_space_pose_.publish(msg);
}
void MarkerTracker::estimatePose(){
	rvecs_.clear();
	tvecs_.clear();
	cv::aruco::estimatePoseSingleMarkers(marker_corners_, marker_size_, camera_matrix_, dist_coeffs_, rvecs_, tvecs_);
}
double calculate2dVecOrientation(cv::Point2f v1, cv::Point2f v2){
	double x = v2.x-v1.x;
	double y = v2.y-v1.y;
	if (x == 0)
		if(y>0)
			return M_PI/2.0;
		else
			return 3*M_PI/2.0;
	if (y == 0)
		if(x>0)
			return 0;
		else
			return M_PI;
	if (y > 0)
		return std::atan2(y,x);
	else
		return M_PI+(M_PI+std::atan2(y,x));
}
void MarkerTracker::estimateMarkerImagePoses(){
	rvecs_.resize(marker_corners_.size());
	tvecs_.resize(marker_corners_.size());
	for (size_t i = 0; i < rvecs_.size(); i++){
		cv::Point2f mid_buttom;
		mid_buttom.x = (marker_corners_[i][0].x + marker_corners_[i][1].x)/2.0;
		mid_buttom.y = (marker_corners_[i][0].y + marker_corners_[i][1].y)/2.0;
		cv::Point2f mid_top;
		mid_top.x = (marker_corners_[i][2].x + marker_corners_[i][3].x)/2.0;
		mid_top.y = (marker_corners_[i][2].y + marker_corners_[i][3].y)/2.0;
		tvecs_[i] = cv::Vec3d((mid_buttom.x + mid_top.x)/2.0, (mid_buttom.y + mid_top.y)/2.0,1);
		rvecs_[i] = cv::Vec3d(0,0,calculate2dVecOrientation(mid_buttom, mid_top));
	}
}
void MarkerTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv::cvtColor(cv_bridge::toCvShare(msg,"bgra8")->image,image_received_,CV_BGRA2BGR);
	new_image_received_ = true;
}
void MarkerTracker::getImage(cv::Mat& img_in){
	img_in = image_received_.clone();
}
void MarkerTracker::sendImage(const cv::Mat& img_out){
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_out).toImageMsg();
	pub_image_.publish(msg);
	cv::waitKey(1);
}
void MarkerTracker::spin(){
	ros::Rate rate(15); // 30 Berk
	while(ros::ok()){
		if(new_image_received_ == true){
			cv::Mat image;
			getImage(image);
			detectMarkers(image);
			estimateMarkerImagePoses();
			drawMarkers(image);
			drawMarkerImagePoses(image);
			publishMarkerPoseImageSpace();
			if(show_image_){
				cv::imshow("Markers",image);
				cv::waitKey(1);
			}
			new_image_received_ = false;
		}
		ros::spinOnce();
		rate.sleep();
	}
}
void MarkerTracker::drawAxis(cv::Mat& image){
	for(size_t i = 0; i < rvecs_.size(); i++){
		cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvecs_[i], tvecs_[i], 0.1);
	}
}
void MarkerTracker::drawMarkerImagePoses(cv::Mat& image){
	for(size_t i = 0; i < rvecs_.size(); i++){
		cv::circle(image, cv::Point(tvecs_[i].val[0],tvecs_[i].val[1]), 3, cv::Scalar(0,0,255), 2, 8);
		cv::line(image, cv::Point(tvecs_[i].val[0],tvecs_[i].val[1]), cv::Point(tvecs_[i].val[0] + 20*cos(rvecs_[i].val[2]),tvecs_[i].val[1] + 20*sin(rvecs_[i].val[2])), cv::Scalar(0,0,255), 2, 8);
	}
}
