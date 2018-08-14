#include "ros/ros.h"
#include "input_coding.hpp"
#include <vector>
#include "common_msgs_gl/SendBool.h"
#include <iostream>
#include <fstream>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/UInt32.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"

class actionPub{
	ros::NodeHandle node_handle_;
	ros::Publisher pub_pressed_key_;
	ros::ServiceServer srvsrvr_enable_, srvsrvr_rerun_path_;
	
	std::vector<double> com_;
	void subscribeTopicsServices();

public:
	actionPub();

	void get_action_path(std::string);

	void Spin(int);

	void printPath();



protected:
	
	void initialize();

	std::vector<std::vector<double>> action_path_;

	int action2key(std::vector<double>);

	bool enable_;

	int path_Iter_;

	bool callbackEnable(common_msgs_gl::SendBool::Request& req, common_msgs_gl::SendBool::Response& res);

	bool callbackRerun(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

}; 
