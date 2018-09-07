/*
1) actuator positions			-	from gripper_state_publisher (/gripper/pos)
2) gripper load				    -	from gripper_state_publisher (/gripper/load)
3) actuators vel ref			-	gripper_node (/gripper_t42/vel_ref_monitor)
4) actuators pos ref			-	gripper_node (/gripper_t42/pos_ref_monitor)
5) finger positions				-   marker_tracker (/marker_tracker/image_space_pose_msg)
6) finger velocities			-   marker_tracker (/marker_tracker/image_space_pose_msg) with numerical diff. - not sure if implemented correctly!!!
7) object position				-	marker_tracker (/marker_tracker/image_space_pose_msg)
8) object orientation			-	marker_tracker (/marker_tracker/image_space_pose_msg)
9) Base pose					- 	marker_tracker (/marker_tracker/image_space_pose_msg)
10) object velocity				-	marker_tracker (/marker_tracker/image_space_pose_msg) with numerical diff. - not sure if implemented correctly!!!
11) obj orientation change		- 	marker_tracker (/marker_tracker/image_space_pose_msg) with numerical diff. - not sure if implemented correctly!!!
12) Actuators velocity			-	from gripper_state_publisher (/gripper/pos)	with numerical diff. - not sure if implemented correctly!!!

1) /camera/image_raw					image
2) ...
3) /gripper/pos							Float32MultiArray
4) ...
5) ...
6) /visual_servoing/vel_ref_monitor		Float64MultiArray
7) ...
8) ...
9) /gripper/curr						Float32MultiArray
10) /gripper/load						Float32MultiArray
11) /gripper_t42/vel_ref_monitor		Float64MultiArray
12) /gripper_t42/pos_ref_monitor		Float64MultiArray

16) curvatures                          Float64MultiArray
17) normals                             Float64MultiArray
18) angles                              PointArray



*/

#include "record_data.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <sys/stat.h>


RecordData::RecordData(){
	if(!node_handle_.getParam("/record_data/requested_data_ids",requested_data_ids_)){
		ROS_WARN("[record_data] The parameter /record_data/requested_data_ids is not set. All the data will be added to the feature vector. This may result in crashes in runtime.");
	}
	if(!node_handle_.getParam("/record_data/marker_no",marker_no_)){
		ROS_WARN("[record_data] The parameter /record_data/marker_no is not set. Marker number is set to %d. This may result in crashes in runtime.", marker_no_);
	}
	if(!node_handle_.getParam("/record_data/data_window_size",data_window_size_)){
		ROS_WARN("[record_data] The parameter /record_data/data_window_size is not set. This parameter is set to %d. This may result in crashes in runtime.", data_window_size_);
	}
	if(!node_handle_.getParam("/record_data/save_images",bool_save_images_)){
		ROS_WARN("[record_data] The parameter /record_data/save_images is not set. This parameter is set to %d. This may result in crashes in runtime.", data_window_size_);
	}
	if(!node_handle_.getParam("/record_data/ref_marker_id",ref_marker_id_)){
		ROS_WARN("[record_data] The parameter /record_data/ref_marker_id is not set. This parameter is set to %d. This may result in crashes in runtime.", ref_marker_id_);
	}
	if(!node_handle_.getParam("/record_data/obj_marker_id",obj_marker_id_)){
		ROS_WARN("[record_data] The parameter /record_data/obj_marker_id is not set. This parameter is set to %d. This may result in crashes in runtime.", obj_marker_id_);
	}

	gripper_pos_.resize(2, 0);
	gripper_load_.resize(2, 0);
	gripper_vel_ref_.resize(2, 0);
	gripper_pos_ref_.resize(2, 0);
	posx_.resize(marker_no_);
	posy_.resize(marker_no_);
	angles_.resize(marker_no_);
	carrot_pos_.resize(2, 0);

//	sub_out_filename_ = node_handle_.subscribe("/play_bags/filename",1,&RecordData::callbackOutFilename, this);
	sub_gripper_pos_ = node_handle_.subscribe("/gripper/pos",1,&RecordData::callbackGripperPos, this);
	sub_gripper_load_ = node_handle_.subscribe("/gripper/load",1,&RecordData::callbackGripperLoad, this);
	sub_gripper_vel_ref_ = node_handle_.subscribe("/gripper_t42/vel_ref_monitor",1,&RecordData::callbackGripperVelRef, this);
	sub_gripper_pos_ref_ = node_handle_.subscribe("/gripper_t42/pos_ref_monitor",1,&RecordData::callbackGripperPosRef, this);
	sub_image_space_pose_ = node_handle_.subscribe("/marker_tracker/image_space_pose_msg",1,&RecordData::callbackMarkerImageSpacePose,this);
	// sub_keyboard_input_ = node_handle_.subscribe("/keyboard_input",1,&RecordData::callbackKeyboardInput,this);
	sub_carrot_ = node_handle_.subscribe("/carrot_point",1,&RecordData::callbackCarrot,this);
	
	// srvsrvr_record_trigger_ = node_handle_.advertiseService("/record_data/record_trigger",&RecordData::callbackRecordTrigger,this);
	// srvsrvr_send_last_data_ = node_handle_.advertiseService("/record_data/send_feature_vector",&RecordData::callbackSendLastData,this);
	srvsrvr_filename_ = node_handle_.advertiseService("/play_bags/filename",&RecordData::callbackFilename,this);

	// image_transport_ = new image_transport::ImageTransport(node_handle_);
	if (bool_save_images_)
		image_sub_ = node_handle_.subscribe("/camera/image_raw", 1, &RecordData::imageCallback, this);

		// sub_image_ = image_transport_->subscribe("/camera/image_raw",1,&RecordData::imageCallback, this);

}

bool RecordData::callbackFilename(common_msgs_gl::SendString::Request& req, common_msgs_gl::SendString::Response& res){
	file_to_write_ = req.data;

	ROS_INFO("[record_data] Now processing file %s", file_to_write_.c_str());
	if(isFileOpen())
		closeFile();
	openFile(file_to_write_);
	return true;
}

// bool RecordData::callbackRecordTrigger(common_msgs_gl::SendInt::Request& req, common_msgs_gl::SendInt::Response& res){
// 	recordLastData(req.data);
// 	return true;
// }
// bool RecordData::callbackSendLastData(common_msgs_gl::GetDoubleArray::Request& req, common_msgs_gl::GetDoubleArray::Response& res){
// 	res.data = constructFeatureVector();
// /*	std::cout<<"begin: ";
// 	for (auto element : res.data)
// 		std::cout<<element<<" ";

// 	std::cout<<std::endl;
// */	return true;
// }

template <typename T>
void pv(std::vector<T> v) {
	for (int i = 0; i < v.size(); i++)
		std::cout << v[i] << " ";
	std::cout << std::endl;
}


std::vector<double> RecordData::constructFeatureVector(){
	std::vector<double> out;
	for (auto element : requested_data_ids_){
		if (element == 1) {
			// std::cout << gripper_pos_[it_gp_].size() << " " << it_gp_ << "\n";
			if (gripper_pos_.size() != 2)	//act pos
				std::runtime_error("[record_data] Gripper pos is not received.");
			
			out.insert(out.end(),gripper_pos_.begin(),gripper_pos_.end());
		}
		else if (element == 2) {
			if (gripper_load_.size() != 2)	//act load
				std::runtime_error("[record_data] Gripper load is not received.");
			out.insert(out.end(),gripper_load_.begin(),gripper_load_.end());
		}
		else if (element == 3) {
			if (gripper_vel_ref_.size() != 2)	//act_vel_ref
				std::runtime_error("[record_data] Gripper vel ref is not received.");
			out.insert(out.end(),gripper_vel_ref_.begin(),gripper_vel_ref_.end());
		}
		else if (element == 4) {
			if (gripper_pos_ref_.size() != 2)	//act_pos_ref
				std::runtime_error("[record_data] Gripper pos ref is not received.");
			out.insert(out.end(),gripper_pos_ref_.begin(),gripper_pos_ref_.end());
		}
		else if (element == 5) {
			if (posx_.size() != marker_no_ || posy_.size() != marker_no_)		// finger pose (x, y, x, y, ...)
				std::runtime_error("[record_data] posx and/or posy is not received.");
			for (size_t i = 0; i < marker_no_; i++){
				if(i == obj_marker_id_ || i == ref_marker_id_)
					continue;
				out.push_back((double)(posx_[i] - 0*posx_[ref_marker_id_]));
				out.push_back((double)(posy_[i] - 0*posy_[ref_marker_id_]));
			}
		}
		// else if (element == 6){		//finger vel
		// 	if (posx_.size() != marker_no_ || posy_.size() != marker_no_)
		// 		std::runtime_error("[record_data] posx or posy is not received.");
		// 	std::vector<double> velx(marker_no_-2), vely(marker_no_-2);
		// 	for (size_t i = 0; i < marker_no_-2; i++){
		// 		if(i == obj_marker_id_ || i == ref_marker_id_)
		// 			continue;
		// 		velx[i] = ((double)(posx_[i] - posx_[nextIt(it_marker_)][i]))/((double)data_window_size_);
		// 		vely[i] = ((double)(posy_[i] - posy_[nextIt(it_marker_)][i]))/((double)data_window_size_);
		// 	}
		// 	out.insert(out.end(),velx.begin(),velx.end());
		// 	out.insert(out.end(),vely.begin(),vely.end());
		// }
		else if (element == 7){ //obj position
			if (posx_.size() != marker_no_)
				std::runtime_error("[record_data] object pos is not received.");
			out.push_back((double)(posx_[obj_marker_id_] - 0*posx_[ref_marker_id_]));
			out.push_back((double)(posy_[obj_marker_id_] - 0*posy_[ref_marker_id_]));
		}
		else if (element == 8){ //obj orientation
			if (angles_.size() != marker_no_)
				std::runtime_error("[record_data] Object angle not received.");
			out.push_back(angles_[obj_marker_id_] - 0*angles_[ref_marker_id_]);
		}
		else if (element == 9){ //base pose
			if (posx_.size() != marker_no_ || posy_.size() != marker_no_ || angles_.size() != marker_no_)
				std::runtime_error("[record_data] Base angle not received.");
			out.push_back(posx_[ref_marker_id_]);
			out.push_back(posy_[ref_marker_id_]);
			out.push_back(angles_[ref_marker_id_]);
		}
		else if (element == 10){ // Keyboard input
			if (key_ == 0)
				std::runtime_error("[record_data] keyboard input not recieved not received.");
			out.push_back(key_);
		}
		else if (element == 11){ //Carrot point
			if (key_ == 0)
				std::runtime_error("[record_data] carrot point not recieved not received.");
			out.push_back(carrot_pos_[0]);
			out.push_back(carrot_pos_[1]);
		}
	// 	else if (element == 10){	//object vel
	// 		if (posx_.size() != marker_no_ || posy_.size() != marker_no_)
	// 			std::runtime_error("[record_data] posx or posy is not received.");
	// 		double velx, vely;
	// 		velx = ((double)(posx_[obj_marker_id_] - posx_[nextIt(it_marker_)][obj_marker_id_]))/((double)data_window_size_);
	// 		vely = ((double)(posy_[obj_marker_id_] - posy_[nextIt(it_marker_)][obj_marker_id_]))/((double)data_window_size_);
	// 		out.push_back(velx);
	// 		out.push_back(vely);
	// 	}
	// 	else if (element == 11){ //obj orientation change
	// 		if (angles_.size() != marker_no_)
	// 			std::runtime_error("[record_data] Object angle not received.");
	// 		out.push_back(((double)(angles_[obj_marker_id_] - angles_[ref_marker_id_])-
	// 						(double)(angles_[nextIt(it_marker_)][obj_marker_id_] - angles_[nextIt(it_marker_)][ref_marker_id_]))/((double)data_window_size_));
	// 	}
	// 	else if (element == 12){	//act vel
	// 		if (gripper_pos_[it_gp_].size() != 2)
	// 			std::runtime_error("[record_data] gripper pos data is not received.");
	// 		std::vector<double> vel(2);
	// 		for (size_t i = 0; i < 2; i++){
	// 			vel[i] = ((double)(gripper_pos_[it_gp_][i] - gripper_pos_[nextIt(it_gp_)][i]))/((double)data_window_size_);
	// 		}
	// 		out.insert(out.end(),vel.begin(),vel.end());
	// 	}
			
	}
	return out;
}


void RecordData::callbackOutFilename(std_msgs::String msg){
	file_to_write_ = msg.data;

	ROS_INFO("[record_data] Now processing file %s", file_to_write_.c_str());
	if(isFileOpen())
		closeFile();
	openFile(file_to_write_);
}
void RecordData::callbackGripperPos(std_msgs::Float32MultiArray msg){
	if (msg.data.size()==0)
		msg.data = {0,0};
	gripper_pos_ = msg.data;
}
void RecordData::callbackGripperLoad(std_msgs::Float32MultiArray msg){
	if (msg.data.size()==0)
		msg.data = {0,0};
	gripper_load_ = msg.data;
}
void RecordData::callbackGripperVelRef(std_msgs::Float64MultiArray msg){
	if (msg.data.size()==0)
		msg.data = {0,0};
	std::vector<double> out;
	out = msg.data;

	// double norm = std::pow(std::pow(out[0],2.0) + std::pow(out[1],2.0),0.5);
	// if(norm == 0){
	// 	out[0] = 0;
	// 	out[1] = 0;
	// }
	// else{
	// 	out[0] = out[0]/norm;
	// 	out[1] = out[1]/norm;
	// }
	// for (size_t i = 0; i < out.size(); i++){ // ???
	// 	if (out[i]>0.3826)
	// 		out[i] = 0.06;
	// 	else if (out[i] < -0.3826)
	// 		out[i] = -0.06;
	// 	else
	// 		out[i] = 0;
	// }

	gripper_vel_ref_ = out;
}
void RecordData::callbackGripperPosRef(std_msgs::Float64MultiArray msg){
	
	if (msg.data.empty() == false)
		msg.data[0] = msg.data[0] + 0.12; // ???
	else
		msg.data = {0,0};
	gripper_pos_ref_ = msg.data;
}

void RecordData::callbackMarkerImageSpacePose(marker_tracker::ImageSpacePoseMsg msg){
	for (size_t i = 0; i < msg.ids.size(); i++){
		posx_[msg.ids[i]] = msg.posx[i];
		posy_[msg.ids[i]] = msg.posy[i];
		angles_[msg.ids[i]] = msg.angles[i];
	}
}

void RecordData::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	image_received_ = cv_bridge::toCvShare(msg,"bgr8")->image;
	first_image_received_ = true;
}

void RecordData::callbackCarrot(std_msgs::Float64MultiArray msg) {
	if (msg.data.size() == 0)
		carrot_pos_ = {0,0};
	carrot_pos_ = msg.data;
}


// void RecordData::callbackKeyboardInput(const std_msgs::Int32 msg){
// 	key_ = msg.data;
// }


/*
bool RecordData::isFinished(){
	if (file_to_write_.compare("-1") == 0)
		return false;
	else
		return true;
}
*/
void RecordData::spin(){
	ros::Rate loop_rate_(15);
	while (ros::ok()){
		ros::spinOnce();
		loop_rate_.sleep();
	}
}
// void RecordData::recordLastData(int target_class){
// 	auto feat_vec = constructFeatureVector();
// 	if (!file_out_.is_open()){
// 		time_t now = time(0);
// 		struct tm * timeinfo;
// 		char str_buffer[80];
// 		time(&now);
// 		timeinfo = localtime(&now);
// 		strftime(str_buffer,sizeof(str_buffer),"%d-%m-%Y %I:%M:%S",timeinfo);
// 		if(!node_handle_.getParam("/record_data/filename",filename_))
// 			ROS_WARN("[record_data] The parameter /record_data/filename is not set. The package %s will be used to read and write files.", filename_.c_str());
// 		if(!node_handle_.getParam("/record_data/file_package_name",package_name_))
// 			ROS_WARN("[record_data] The parameter /record_data/file_package_name is not set. The package %s will be used to read and write files.", package_name_.c_str());

// 		std::string path = ros::package::getPath(package_name_);
// 		path_out_ = path + "/"+ filename_+str_buffer;
// 		file_out_.open(path_out_ + ".txt");
// 		if (bool_save_images_){
// 			if(mkdir(path_out_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1){
// 				ROS_ERROR("[record_data] The output file directory cannot be open.");
// 			}
// 		}
				
// 	}
// 	for	(auto element : feat_vec){
// 		file_out_<<element;
// 		file_out_<<',';
// 	}
// 	file_out_<<target_class;
// 	file_out_<<"\n";
// 	// if (bool_save_images_ && first_image_received_){
// 	// 	saved_data_index_++;
// 	// 	imwrite( path_out_ + "/image" + std::to_string(saved_data_index_) + ".jpg", image_received_ );
// 	// 	std::cout<<"Saved data point number: "<<saved_data_index_<<std::endl;
// 	// }
// }

void RecordData::openFile(std::string filename_){
	time_t now = time(0);
	struct tm * timeinfo;
	char str_buffer[80];
	time(&now);
	timeinfo = localtime(&now);
	strftime(str_buffer,sizeof(str_buffer),"_%d-%m-%Y %I:%M:%S",timeinfo);
	if(!node_handle_.getParam("/record_data/destination",destination_))
		ROS_WARN("[record_data] The parameter /record_data/destination is not set. The package %s will be used to read and write files.", destination_.c_str());

	std::string path = destination_;
	path_out_ = path + "/"+ filename_;// + str_buffer;
	file_out_.open(path_out_ + ".txt");
	if (bool_save_images_){
		if(mkdir(path_out_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1){
			ROS_ERROR("[record_data] The output file directory cannot be open.");
		}
	}

	startTime_ = std::chrono::high_resolution_clock::now();
}

bool RecordData::isFileOpen(){
	return file_out_.is_open();
}

void RecordData::continuousRecord(int record_freq){
	ros::Rate loop_rate_(record_freq);
//	waitForMessages();
	while (ros::ok()){
		if (file_to_write_.length() == 0){
			ros::spinOnce();
			loop_rate_.sleep();
			continue;
		}
		else if (file_to_write_.compare("-1") == 0)
			break;
		if(isFileOpen()){
			auto feat_vec = constructFeatureVector();
			auto curr_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startTime_).count();
			file_out_ << curr_time;
			for	(auto element : feat_vec){
				file_out_ << ' ';
				file_out_ << element;
			}
			file_out_<< "\n";
			if (bool_save_images_ && first_image_received_){
				saved_data_index_++;
				std::string file = path_out_ + "/image_test3_" + std::to_string(saved_data_index_) + "_" + std::to_string(int(floor(curr_time*100))) +".jpg";
				imwrite( file , image_received_ );
				std::cout << "Saved image of data point number " << saved_data_index_ << " to " << file << std::endl;
			}
		}
		ros::spinOnce();
		loop_rate_.sleep();
	}
}

void RecordData::waitForMessages(){
	ros::topic::waitForMessage<std_msgs::Float32MultiArray>("gripper_pos", node_handle_, ros::Duration(10));
}

void RecordData::closeFile(){
	file_out_.close();
}
RecordData::~RecordData(){
	closeFile();
}
