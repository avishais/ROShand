#include "marker_tracker/marker_tracker.hpp"

int main (int argc, char** argv){
	ros::init(argc, argv,"marker_tracker");
	MarkerTracker mt;
	mt.spin();
	return 0;
}
