//#define DYNAMIXEL_XM
#include "test.hpp"
#include "std_msgs/Float64MultiArray.h"
#include "common_msgs_gl/SendDoubleArray.h"
#include "common_msgs_gl/GetDoubleArray.h"
#include "common_msgs_gl/SendIntArray.h"
test::test() : node_handle_("~"){
	initialize();
}

void test::initialize(){
	// readParametersFromServer();
	subscribeTopicsServices();
}


void test::subscribeTopicsServices(){
	pub_m_ = node_handle_.advertise<std_msgs::Float64MultiArray>("myTopic",1);
}


void test::sendCommand(std::vector<double> commands){
	std_msgs::Float64MultiArray msg;
	msg.data.assign(commands.begin(), commands.end());
	pub_m_.publish(msg);

}
