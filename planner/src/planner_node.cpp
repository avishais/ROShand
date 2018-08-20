
#include "plan.hpp"

int main (int argc, char** argv){
	ros::init(argc, argv,"planner_node");
    ros::Time::init();
	plan_hand g;

    g.Spin();

    // Vector start = {0,0,0,0};
    // Vector goal = {0.5,0.4,0.9,0.1};

    // g.plan(start, goal, 10, PLANNER_RRT);

	return 0;
}
