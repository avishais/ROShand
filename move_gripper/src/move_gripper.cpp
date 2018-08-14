//#define DYNAMIXEL_XM
#include "move_gripper.hpp"
#include "openhand/ReadServos.h"
#include "std_msgs/Float64MultiArray.h"
#include "common_msgs_gl/SendDoubleArray.h"
#include "common_msgs_gl/GetDoubleArray.h"
#include "common_msgs_gl/SendIntArray.h"

MoveGripper::MoveGripper() : node_handle_("~"){
	initialize();
}

void MoveGripper::initialize(){
	subscribeTopicsServices();
}

void MoveGripper::subscribeTopicsServices(){
#ifndef DYNAMIXEL_XM
	srvclnt_send_gripper_commands_ = node_handle_.serviceClient<openhand::MoveServos>("/MoveServos");
	srvclnt_read_gripper_data_ = node_handle_.serviceClient<openhand::ReadServos>("/ReadServos");
#else
	srvclnt_send_gripper_commands_ = node_handle_.serviceClient<common_msgs_gl::SendDoubleArray>("/cmd_pos");
	srvclnt_read_gripper_data_ = node_handle_.serviceClient<common_msgs_gl::GetDoubleArray>("/read_pos");
	srvclnt_set_operating_mode_ = node_handle_.serviceClient<common_msgs_gl::SendIntArray>("/set_operating_mode");
#endif

    sub_pos_ref_ = node_handle_.subscribe("/gripper_t42/pos_ref_monitor", 1, &MoveGripper::callbackGripperPose, this);
    sub_vel_ref_ = node_handle_.subscribe("/gripper_t42/vel_ref_monitor", 1, &MoveGripper::callbackGripperVel, this);
    // pub_pos_ref_ = node_handle_.advertise<std_msgs::Float64MultiArray>("pos_ref_cat",1);
	// pub_vel_ref_ = node_handle_.advertise<std_msgs::Float64MultiArray>("vel_ref_cat",1);

}


void MoveGripper::callbackGripperPose(std_msgs::Float64MultiArray msg) {
    pos_ref_.resize(msg.data.size());
	for(size_t i = 0; i < msg.data.size(); i++){
		pos_ref_[i] = msg.data[i];
	}

    // pub_pos_ref_.publish(msg);
}

void MoveGripper::callbackGripperVel(std_msgs::Float64MultiArray msg) {
    vel_ref_.resize(msg.data.size());
	for(size_t i = 0; i < msg.data.size(); i++){
		vel_ref_[i] = msg.data[i];
	}

	// pub_vel_ref_.publish(msg);
}



bool MoveGripper::velMode(){
	return velocity_mode_on_;
}


void MoveGripper::sendCommand(std::vector<double> commands){

#ifndef DYNAMIXEL_XM
	openhand::MoveServos srv;
	srv.request.pos.assign(commands.begin(), commands.end());
#else
	common_msgs_gl::SendDoubleArray srv;
	srv.request.data.assign(commands.begin(), commands.end());
#endif

	int trial_no = 0;
	while(!srvclnt_send_gripper_commands_.call(srv) && ros::ok()){
		trial_no++;
		if(trial_no>3){
			//throw std::runtime_error("[gripper_m2]: The service for sending position commands to gripper motors has failed."); 
		}
	}
}

void MoveGripper::Move() {

    sendCommand(pos_ref_);

}


void MoveGripper::spin(int frequency) {
    ros::Rate loop_rate(frequency);
    while(ros::ok()){
		// if(g.velMode()){
		// 	g.velocityModeStep();	
		// }
        Move();
		ros::spinOnce();
		loop_rate.sleep();
}
}




