#include "action.hpp"

int main (int argc, char** argv){
	ros::init(argc, argv,"action_node");
	actionPub g;
	g.Spin(0.8);
	return 0;
}
