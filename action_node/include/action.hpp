#include "ros/ros.h"
#include "input_coding.hpp"
#include <vector>
#include "common_msgs_gl/SendBool.h"
#include "marker_tracker/ImageSpacePoseMsg.h"
#include <iostream>
#include <fstream>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt32.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include "planner/plan_req.h"
#include "action_node/empty.h"

class actionPub{
	ros::NodeHandle node_handle_;
	ros::Publisher pub_pressed_key_;
	ros::ServiceServer srvsrvr_enable_, srvsrvr_rerun_path_;
	ros::Subscriber sub_markers_, sub_load_, sub_pos_;
	ros::ServiceClient plan_client_, plot_client_;
	
	std::vector<double> com_;
	void subscribeTopicsServices();

public:
	actionPub();

	void get_action_path(std::string);

	void Spin(double);

	void printPath();

protected:
	
	void initialize();

	std::vector<std::vector<double>> action_path_;

	int action2key(std::vector<double>);

	void callPlan();

	bool enable_;

	bool got_plan_, got_obj_pos_, got_load_, loaded_path_;

	bool should_replan; // Just a manual flag to perform replan or take path from file

	int path_Iter_;

	bool callbackEnable(common_msgs_gl::SendBool::Request& req, common_msgs_gl::SendBool::Response& res);

	bool callbackRerun(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

	void callbackObjectPos(marker_tracker::ImageSpacePoseMsg msg);
	void callbackGripperLoad(std_msgs::Float32MultiArray msg);
	void callbackGripperPos(std_msgs::Float32MultiArray msg);
	std::vector<double> obj_pos_;
	std::vector<double> gripper_load_;
	std::vector<double> gripper_pos_;

}; 
