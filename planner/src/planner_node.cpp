#include "plan.hpp"

int main (int argc, char** argv){
	ros::init(argc, argv,"planner_node");
	plan_hand g;
	g.Spin(15);
	return 0;
}
