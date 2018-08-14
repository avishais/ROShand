#include "ros/ros.h"
#include "record_data.hpp"
int main(int argc, char** argv){
	ros::init(argc,argv,"record_data");
	RecordData RD;
	
//	while (ros::ok()){
		try{
			RD.continuousRecord(15);
		}
		catch (std::runtime_error e){
			ROS_WARN("[record_data] Error while continuously recording the data: %s", e.what());
			sleep(0.1);
		}
/*		if(ml_preprocessor.isFinished())
			break;
*///	}
	RD.closeFile();
	return 0;
}
