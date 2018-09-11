//#define DYNAMIXEL_XM
#include "play_actions.hpp"

playAct::playAct() : node_handle_("~"){
	initialize();

	enable_ = false;
	loaded_path_ = false;
}

void playAct::initialize(){
	// readParametersFromServer();
	subscribeTopicsServices();
	
	obj_pos_.resize(2);
}


void playAct::subscribeTopicsServices(){
	pub_pressed_key_ = node_handle_.advertise<std_msgs::UInt32>("/keyboard_input",1);
	srvsrvr_enable_ = node_handle_.advertiseService("/play_actions/enable",&playAct::callbackEnable,this);
	srvsrvr_rerun_path_ = node_handle_.advertiseService("/play_actions/rerun_path",&playAct::callbackRerun,this);
	plot_client_ = node_handle_.serviceClient<action_node::empty>("/plot_ref_path");

	sub_markers_ = node_handle_.subscribe("/marker_tracker/image_space_pose_msg", 1000, &playAct::callbackObjectPos, this);
}

bool playAct::callbackEnable(common_msgs_gl::SendBool::Request& req, common_msgs_gl::SendBool::Response& res) {
	enable_ = req.data;
	return true;
}

bool playAct::callbackRerun(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	path_Iter_ = 0;
	return true;
}

void playAct::get_action_path(std::string file) {

	ROS_INFO("[play_actions] Loading path data...");

	std::ifstream myfile;
    myfile.open("/home/pracsys/catkin_ws/src/rutgers_collab/src/play_actions_node/paths/" + file);

	if (!myfile)
    {
        ROS_ERROR("[play_actions] Failed to open action_path file.");
    }

	std::vector<double> a(2);
	while(!myfile.eof()) {
		myfile >> a[0];
		myfile >> a[1];
		action_path_.push_back(a);
	}
    myfile.close();

	ROS_INFO("[play_actions] Action_path file loaded, ready to execute");

	loaded_path_ = true;

	// for (int i = 0; i < action_path_.size(); i++) 
	// 	std::cout << action_path_[i][0] << " " << action_path_[i][1] << std::endl;
}

void playAct::Spin(double frequency) {

	ros::Rate loop_rate(frequency);
	
	while(ros::ok()){	

        if (!loaded_path_)
            get_action_path("actionPath.txt");

        if (loaded_path_ && enable_ && path_Iter_ < action_path_.size()) {
            ROS_INFO("[action_node] action path node number %d: <%.2f,%.2f>.", path_Iter_, action_path_[path_Iter_][0], action_path_[path_Iter_][1]);	

            std_msgs::UInt32 msg;
            msg.data = action8_2key(action_path_[path_Iter_]);		
            
            ROS_INFO("[action_node] publishing action key %d.", msg.data);
            pub_pressed_key_.publish(msg);

            path_Iter_++;
        }

        if (path_Iter_ == action_path_.size()) {
            ROS_INFO("[action_node] Reached end of path");
            path_Iter_++;
        }

        if (path_Iter_ > action_path_.size()) {
            std_msgs::UInt32 msg;
            msg.data = action8_2key({0,0});	

            pub_pressed_key_.publish(msg);
        }

		ros::spinOnce();
		loop_rate.sleep();
	}
}

int playAct::action2key(std::vector<double> a) {
	
	if (a[0] < 0 && a[1] < 0) // Down
		return KEY_W;

	if (a[0] > 0 && a[1] > 0) // Up
		return KEY_X;
	
	if (a[0] < 0 && a[1] > 0) // Left
		return KEY_D;

	if (a[0] > 0 && a[1] < 0) // right
		return KEY_A;

	if (a[0] == 0 && a[1] == 0)
		return KEY_S;

}

int playAct::action8_2key(std::vector<double> a) {
	
	if (a[0] == 0 && a[1] == -0.4) // up-left
		return KEY_Q;

	if (a[0] == -0.4 && a[1] == 0) // up-right
		return KEY_E;
	
	if (a[0] == -0.2 && a[1] == -0.2) // up
		return KEY_W;

	if (a[0] == -0.2 && a[1] == 0.2) // right
		return KEY_D;

	if (a[0] == 0.2 && a[1] == -0.2) // left
		return KEY_A;

	if (a[0] == 0.2 && a[1] == 0.2) // down
		return KEY_X;	

	if (a[0] == 0 && a[1] == 0.4) // down-right
		return KEY_C;

	if (a[0] == 0.4 && a[1] == 0) // down-left
		return KEY_Z;

	if (a[0] == 0 && a[1] == 0)
		return KEY_S;

}

void playAct::printPath() {

	for (int i = 0; i < action_path_.size(); i++)
		ROS_INFO("[action_node] action path node number %d: <%f,%f>.", i, action_path_[i][0], action_path_[i][1]);

}

void playAct::callbackObjectPos(marker_tracker::ImageSpacePoseMsg msg) {
    std::vector<double> posx(6);
    std::vector<double> posy(6);
    std::vector<double> angles(6);
    
    for (size_t i = 0; i < msg.ids.size(); i++){
		posx[msg.ids[i]] = msg.posx[i];
		posy[msg.ids[i]] = msg.posy[i];
		angles[msg.ids[i]] = msg.angles[i];
	}

    double base_theta = 3.14159265359 - angles[0];

    obj_pos_ = {posx[5]-posx[0], posy[5]-posy[0]}; // Substract the base position

	// std::cout << "-----\n";
	// std::cout << "B.obj_pos_: " << posx[5] << " " << posy[5] << std::endl;
	// std::cout << "B.base: " << posx[0] << " " << posy[0] << " " << 3.14159265359-angles[0] << std::endl;

    obj_pos_[0] = obj_pos_[0]*cos(base_theta) - obj_pos_[1]*sin(base_theta);
    obj_pos_[1] = obj_pos_[0]*sin(base_theta) + obj_pos_[1]*cos(base_theta);

	// std::cout << "A.obj_pos_: " << obj_pos_[0] << " " << obj_pos_[1] << std::endl;

	got_obj_pos_ = true;
}


