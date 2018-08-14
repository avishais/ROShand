#include "test.hpp"

int main (int argc, char** argv){
	ros::init(argc, argv,"test_node");
	test g;
	ros::Rate loop_rate(100);
	std::vector<double> c(2, 4.);
	while(ros::ok()){		
		g.sendCommand(c);

		// ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
