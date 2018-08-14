#include "gripper_state_publisher.hpp"

int main(int argc, char** argv){
	ros::init(argc, argv,"grippper_state_publisher");

	GripperStatePublisher gsp(10);

	gsp.spin();

	return 0;
}
