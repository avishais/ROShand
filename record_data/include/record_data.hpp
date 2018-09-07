#ifndef ML_PREPROCESSOR_HPP
#define ML_PREPROCESSOR_HPP
#include <fstream>
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "marker_tracker/ImageSpacePoseMsg.h"
#include "common_msgs_gl/SendInt.h"
#include "common_msgs_gl/GetDoubleArray.h"
#include "common_msgs_gl/PointArray.h"
#include "common_msgs_gl/SendString.h"
#include <chrono>

class RecordData{
	int saved_data_index_ = 0;
	int data_window_size_ = 10;
	int marker_no_ = 6;
	int ref_marker_id_ = 0;
	int obj_marker_id_ = 5;
	bool first_image_received_ = false;
	bool bool_save_images_;
	std::string path_out_, file_to_write_;
	std::string destination_ = "./";
	std::string filename_ = "training_data/training_data";
	int key_ = 0;
	std::vector<double> carrot_pos_;
	std::vector<int> posx_;
	std::vector<int> posy_;
	std::vector<double> angles_;
	std::vector<float> gripper_pos_, gripper_load_;
	std::vector<double> gripper_vel_ref_, gripper_pos_ref_;
	std::vector<int> requested_data_ids_ = {1,2,3,4,5,6,7,8,9,10};
	std::ofstream file_out_;
	int it_gp_ = data_window_size_-1,it_ca_ = data_window_size_-1, it_gl_ = data_window_size_-1, it_cur_ = data_window_size_-1, it_normal_ = data_window_size_-1, it_vr_ = data_window_size_-1, it_pr_ = data_window_size_-1, it_cvr_ = data_window_size_-1, it_marker_ = data_window_size_-1;
	std::chrono::high_resolution_clock::time_point startTime_;
	cv::Mat image_received_;
	ros::NodeHandle node_handle_;
	image_transport::ImageTransport * image_transport_;
	image_transport::Subscriber sub_image_;
	ros::Subscriber sub_image_space_pose_, sub_gripper_pos_, sub_gripper_load_, sub_gripper_vel_ref_, sub_gripper_pos_ref_, sub_cartesian_vel_ref_, sub_curvatures_, sub_normals_, sub_contact_angles_, sub_out_filename_, image_sub_, sub_keyboard_input_, sub_carrot_;
	ros::ServiceServer srvsrvr_record_trigger_, srvsrvr_send_last_data_, srvsrvr_filename_;
	void increaseIt(int& it);
	int nextIt(int it);
	void recordLastData(int);
	std::vector<double> constructFeatureVector();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void callbackMarkerImageSpacePose(marker_tracker::ImageSpacePoseMsg msg);
	void callbackGripperPos(std_msgs::Float32MultiArray msg);
	void callbackGripperLoad(std_msgs::Float32MultiArray msg);
	void callbackGripperVelRef(std_msgs::Float64MultiArray msg);
	void callbackGripperPosRef(std_msgs::Float64MultiArray msg);
	void callbackCarrot(std_msgs::Float64MultiArray msg);
	bool callbackRecordTrigger(common_msgs_gl::SendInt::Request& req, common_msgs_gl::SendInt::Response& res);
	bool callbackSendLastData(common_msgs_gl::GetDoubleArray::Request& req, common_msgs_gl::GetDoubleArray::Response& res);
	bool callbackFilename(common_msgs_gl::SendString::Request& req, common_msgs_gl::SendString::Response& res);
	void callbackOutFilename(std_msgs::String msg);
	// void callbackKeyboardInput(const std_msgs::Int32 msg);
	void waitForMessages();
public:
	RecordData();
	void spin();
	void openFile(std::string);
	bool isFileOpen();
	void closeFile();
	void continuousRecord(int record_freq = 30);
	~RecordData();
};


#endif	// ML_PREPROCESSOR_HPP
