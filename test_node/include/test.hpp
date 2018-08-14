#include "ros/ros.h"
#include "openhand/MoveServos.h"
#include <vector>
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
class test{
	ros::Publisher pub_m_;
	
	std::vector<double> com_;
	void subscribeTopicsServices();

public:
	test();
	void sendCommand(std::vector<double>);


protected:
	ros::NodeHandle node_handle_;

	void initialize();
}; 
