//#define DYNAMIXEL_XM
#include "action.hpp"

actionPub::actionPub() : node_handle_("~"){
	initialize();

	enable_ = false;
	path_Iter_ = 0;
}

void actionPub::initialize(){
	// readParametersFromServer();
	subscribeTopicsServices();
	
	obj_pos_.resize(2);
    gripper_load_.resize(2);
    gripper_pos_.resize(2);

	callPlan();

	get_action_path("path.txt");
}


void actionPub::subscribeTopicsServices(){
	pub_pressed_key_ = node_handle_.advertise<std_msgs::UInt32>("/keyboard_input",1);
	srvsrvr_enable_ = node_handle_.advertiseService("/action/enable",&actionPub::callbackEnable,this);
	srvsrvr_rerun_path_ = node_handle_.advertiseService("/action/rerun_path",&actionPub::callbackRerun,this);
	plan_client_ = node_handle_.serviceClient<planner::plan_req>("/plan_hand/plan");

	sub_markers_ = node_handle_.subscribe("/marker_tracker/image_space_pose_msg", 1000, &actionPub::callbackObjectPos, this);
    sub_load_ = node_handle_.subscribe("/gripper/load", 1000, &actionPub::callbackGripperLoad, this);
    sub_pos_ = node_handle_.subscribe("/gripper/pos", 1000, &actionPub::callbackGripperPos, this);
}

bool actionPub::callbackEnable(common_msgs_gl::SendBool::Request& req, common_msgs_gl::SendBool::Response& res) {
	enable_ = req.data;
	return true;
}

bool actionPub::callbackRerun(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	path_Iter_ = 0;
	return true;
}


void actionPub::get_action_path(std::string file) {

	std::ifstream myfile;
    myfile.open("/home/pracsys/catkin_ws/src/rutgers_collab/src/planner/paths/" + file);

	if (!myfile)
    {
        ROS_ERROR("[action_node] Failed to open action_path file.");
    }

	std::vector<double> a(2);
	while(!myfile.eof()) {
		myfile >> a[0];
		myfile >> a[1];
		action_path_.push_back(a);
	}
    myfile.close();

	ROS_INFO("[action_node] Action_path file loaded.");

	for (int i = 0; i < action_path_.size(); i++) 
		std::cout << action_path_[i][0] << " " << action_path_[i][1] << std::endl;
}

void actionPub::Spin(int frequency) {

	ros::Rate loop_rate(frequency);
	
	while(ros::ok()){	

		if (enable_ && path_Iter_ < action_path_.size()) {
			ROS_INFO("[action_node] action path node number %d: <%f,%f>.", path_Iter_, action_path_[path_Iter_][0], action_path_[path_Iter_][1]);	

			std_msgs::UInt32 msg;
			msg.data = action2key(action_path_[path_Iter_]);		
			
			pub_pressed_key_.publish(msg);

			path_Iter_++;
		}

		if (path_Iter_ == action_path_.size()) {
			ROS_INFO("[action_node] Reached end of path");
			path_Iter_++;
		}

		if (path_Iter_ > action_path_.size()) {
			std_msgs::UInt32 msg;
			msg.data = action2key({0,0});	

			pub_pressed_key_.publish(msg);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

int actionPub::action2key(std::vector<double> a) {
	
	if (a[0] < 0 && a[1] < 0) // Down
		return KEY_W;

	if (a[0] > 0 && a[1] > 0) // Up
		return KEY_X;
	
	if (a[0] < 0 && a[1] > 0) // Left
		return KEY_A;

	if (a[0] > 0 && a[1] < 0) // right
		return KEY_D;

	if (a[0] == 0 && a[1] == 0)
		return KEY_S;

}

void actionPub::printPath() {

	for (int i = 0; i < action_path_.size(); i++)
		ROS_INFO("[action_node] action path node number %d: <%f,%f>.", i, action_path_[i][0], action_path_[i][1]);

}

void actionPub::callPlan() {

	std::vector<double> state = obj_pos_;
	state.insert( state.end(), gripper_load_.begin(), gripper_load_.end() );

	planner::plan_req srv;
	srv.request.start = {0, -400, 0, 0};//state;
	srv.request.goal = {300, -100 , 200, 0};

	plan_client_.call(srv);
}

void actionPub::callbackObjectPos(marker_tracker::ImageSpacePoseMsg msg) {
    std::vector<double> posx(msg.ids.size());
    std::vector<double> posy(msg.ids.size());
    std::vector<double> angles(msg.ids.size());
    
    for (size_t i = 0; i < msg.ids.size(); i++){
		posx[msg.ids[i]] = msg.posx[i];
		posy[msg.ids[i]] = msg.posy[i];
		angles[msg.ids[i]] = msg.angles[i];
	}

    double base_theta = 3.14159265359 - angles[0];

    obj_pos_ = {posx[5]-posx[0], posy[5]-posx[0]}; // Substract the base position

    obj_pos_[0] = obj_pos_[0]*cos(base_theta) - obj_pos_[1]*sin(base_theta);
    obj_pos_[1] = obj_pos_[0]*sin(base_theta) + obj_pos_[1]*cos(base_theta);
    
}

void actionPub::callbackGripperLoad(std_msgs::Float64MultiArray msg) {
    gripper_load_[0] = msg.data[0];
    gripper_load_[1] = msg.data[1];
}

void actionPub::callbackGripperPos(std_msgs::Float64MultiArray msg) {
    gripper_pos_[0] = msg.data[0];
    gripper_pos_[1] = msg.data[1];
}


