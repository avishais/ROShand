#include "ros/ros.h"
#include "std_msgs/String.h"
#include "common_msgs_gl/SendString.h"
int main(int argc, char** argv){
	ros::init(argc,argv,"play_bags");
	ros::NodeHandle node_handle;
	std::string bag_directory;
	std::vector<std::string> bag_files;
	
	ros::ServiceClient srvclnt_filename = node_handle.serviceClient<common_msgs_gl::SendString>("/play_bags/filename",0);

	if(!node_handle.getParam("/play_bags/bag_directory",bag_directory)){
		std::runtime_error("[play_bags] The parameter /play_bags/bag_directory is not set." );
	}
	if(!node_handle.getParam("/play_bags/bag_files",bag_files)){
		std::runtime_error("[play_bags] The parameter /play_bags/bag_files is not set." );
	}
	ROS_INFO("[play_bags] Waiting for the play to get ready.");

	ros::Duration(5).sleep();
	for (auto file : bag_files){
		common_msgs_gl::SendString srv_filename;
		srv_filename.request.data = file;
		srvclnt_filename.call(srv_filename);
		ros::spinOnce();
		ros::Duration(15).sleep();
		std::string system_command = "rosbag play " + bag_directory + "/" + file + ".bag"; 
		ROS_INFO("[play_bags] Playing the file %s", file.c_str());
		system(system_command.c_str());
	}

	common_msgs_gl::SendString srv_filename;
	srv_filename.request.data = "-1";
	srvclnt_filename.call(srv_filename);
	ros::spinOnce();

	return 0;
}
