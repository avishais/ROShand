//#define DYNAMIXEL_XM

#include "gripper_state_publisher.hpp"
#include "openhand/ReadServos.h"
#include "openhand/ReadLoad.h"
#include "openhand/ReadCurrent.h"
#include "std_msgs/Float32MultiArray.h"
#include "common_msgs_gl/GetDoubleArray.h"

GripperStatePublisher::GripperStatePublisher(double frequency):loop_rate_(frequency) {
#ifndef DYNAMIXEL_XM
	srv_clnt_pos_ = n_.serviceClient<openhand::ReadServos>("/ReadServos");
#else
	srv_clnt_pos_ = n_.serviceClient<common_msgs_gl::GetDoubleArray>("/read_pos");
#endif
#ifndef DYNAMIXEL_XM
	srv_clnt_load_ = n_.serviceClient<openhand::ReadLoad>("/ReadLoad");
#else
	srv_clnt_load_ = n_.serviceClient<common_msgs_gl::GetDoubleArray>("/read_current");
#endif
	srv_clnt_current_ = n_.serviceClient<openhand::ReadCurrent>("/ReadCurrent");
	pub_pos_ = n_.advertise<std_msgs::Float32MultiArray>("/gripper/pos",1);
	pub_load_ = n_.advertise<std_msgs::Float32MultiArray>("/gripper/load",1);
	pub_curr_ = n_.advertise<std_msgs::Float32MultiArray>("/gripper/curr",1);
}

void GripperStatePublisher::spin(){
	
	while(ros::ok()){

#ifndef DYNAMIXEL_XM
		openhand::ReadServos srv_pos;
#else
		common_msgs_gl::GetDoubleArray srv_pos;
#endif

#ifndef DYNAMIXEL_XM
		openhand::ReadLoad srv_load;
#else
		common_msgs_gl::GetDoubleArray srv_load;
#endif
		openhand::ReadCurrent srv_current;
	
#ifndef DYNAMIXEL_XM
		srv_clnt_pos_.call(srv_pos);
#else 
		srv_clnt_pos_.call(srv_pos);
#endif
		srv_clnt_load_.call(srv_load);
		srv_clnt_current_.call(srv_current);

		std_msgs::Float32MultiArray position;
#ifndef DYNAMIXEL_XM
		position.data = srv_pos.response.pos;
#else
		std::vector<float> position_temp(srv_pos.response.data.begin(), srv_pos.response.data.end());
		position.data = position_temp;
#endif
		std_msgs::Float32MultiArray load;

#ifndef DYNAMIXEL_XM
		load.data = srv_load.response.load;
#else
		std::vector<float> load_temp(srv_load.response.data.begin(), srv_load.response.data.end());
		load.data = load_temp;
#endif
		std_msgs::Float32MultiArray current;
		current.data = srv_current.response.curr;
		pub_pos_.publish(position);
		pub_load_.publish(load);
		pub_curr_.publish(current);
		ros::spinOnce();
		loop_rate_.sleep();
	}
	
}
