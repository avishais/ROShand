#include "marker_tracker/marker_generator.hpp"
#include <opencv2/highgui.hpp>
#include <ros/package.h>
MarkerGenerator::MarkerGenerator(){
	path_ = ros::package::getPath("marker_tracker");
}

cv::Mat MarkerGenerator::generate6x6Marker(int marker_id, int marker_size, int border_thickness){
	cv::Mat marker_image;
	if(marker_id<0 || marker_id>249){
		printf("\n [MarkerGenerator] The marker id should be between 0 and 249. The given id is %d", marker_id);
		return marker_image;
	}
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::aruco::drawMarker(dictionary, marker_id, marker_size, marker_image, border_thickness);
	return marker_image;
}

void MarkerGenerator::saveMarker(std::string name, cv::Mat marker_image){
	cv::imwrite(path_+"/markers/" + name, marker_image);
}
