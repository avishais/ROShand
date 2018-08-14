#include "ros/ros.h"
class GripperStatePublisher{

	ros::NodeHandle n_;
	ros::ServiceClient srv_clnt_pos_, srv_clnt_load_, srv_clnt_current_;
	ros::Publisher pub_pos_, pub_curr_, pub_load_, pub_pos_ref_;
	ros::Rate loop_rate_;
public:
	GripperStatePublisher(double frequency);
	void spin();	
};
