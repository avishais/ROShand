#include "play_actions.hpp"

int main (int argc, char** argv){
	ros::init(argc, argv,"play_actions_node");
	playAct g;
	g.Spin(15);
	return 0;
}
