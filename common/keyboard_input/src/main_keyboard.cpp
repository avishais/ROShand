#include "keyboard_input/keyboard.hpp"
int main(int argc, char** argv){
	ros::init(argc, argv, "keyboard_input");	
	Keyboard k(15);
	k.spin();

	return 0;
}
