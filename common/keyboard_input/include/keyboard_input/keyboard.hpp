#include "ros/ros.h"
#include "termios.h"
#include "common_msgs_gl/SendBool.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>
#include <stdio.h>

#include "input_coding.hpp"

class Keyboard{
	bool enable_ = true;
	bool enable_auto_ = false;
	ros::NodeHandle node_handle_;
	ros::Publisher pub_pressed_key_;
	ros::ServiceServer srvsrvr_enable_, srvsrvr_auto_;
	ros::Subscriber sub_gripper_load_;
	int kfd_ = 0;
	struct termios cooked_, raw_;
	double loop_frequency_;
	std::vector<float> cur_gripper_load_;
	int last_action = 0;
	std::vector<int> pos_keys_ = {113, 119, 101, 97, 100, 122, 120, 99}; 
public:
	Keyboard(double loop_frequency = 10);
	~Keyboard();
	void spin();
private:
	unsigned int readKey();
	void initialize();
	void publishPressedKey(unsigned int value);
	bool callbackEnable(common_msgs_gl::SendBool::Request& req, common_msgs_gl::SendBool::Response& res);
	bool callbackAuto(common_msgs_gl::SendBool::Request& req, common_msgs_gl::SendBool::Response& res);
	void callbackRefresehGripperLoad(std_msgs::Float32MultiArray msg);
	int random_key();
	void auto_collect();

};
