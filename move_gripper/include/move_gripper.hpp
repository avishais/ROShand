#include "ros/ros.h"
#include "openhand/MoveServos.h"
#include <vector>
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64MultiArray.h"
#include "gripper_nodes/CommandTrajectory.h"
#include "common_msgs_gl/SendDoubleArray.h"
class MoveGripper {
	std::vector<double> finger_initial_offset_, finger_closing_position_, finger_opening_position_;
	ros::ServiceClient srvclnt_send_gripper_commands_;
	ros::ServiceClient srvclnt_read_gripper_data_, srvclnt_set_operating_mode_;

	ros::Subscriber sub_pos_ref_, sub_vel_ref_;
	ros::Publisher pub_pos_ref_, pub_vel_ref_;

	bool velocity_mode_on_ = false;
	bool initial_motor_pos_set_ = false;
	std::vector<float> vel_cont_init_motor_pos_;
	std::vector<float> motor_pos_ref_;
	std::vector<double> vel_ref_;
    std::vector<double> pos_ref_;

	void subscribeTopicsServices();
public:
	MoveGripper();
	bool velMode();
	void velocityModeStep();
	void spin(int frequency = 100);
protected:
	ros::NodeHandle node_handle_;
	bool stop_trigger_ = false, executing_trajectory_ = false;
	void sendCommand(std::vector<double>);
    void initialize();
   	void callbackGripperPose(std_msgs::Float64MultiArray msg);
	void callbackGripperVel(std_msgs::Float64MultiArray msg);
	void Move();

}; 
