#include "gripper.hpp"

int main (int argc, char** argv){
	ros::init(argc, argv,"gripper_node");
	Gripper g;
	ros::Rate loop_rate(15);
	while(ros::ok()){
		if(g.velMode()){
			g.velocityModeStep();	
		}
		else
			g.publishRefZero();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
