#include "move_gripper.hpp"

int main (int argc, char** argv){
	ros::init(argc, argv,"move_gripper");
	MoveGripper g;
	g.spin(15);
	
	return 0;
}
