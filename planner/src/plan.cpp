
/* Author: Avishai Sintov */

#include "plan.hpp"

plan_hand::plan_hand() : node_handle_("~") {

    subscribeTopicsServices();

}

void plan_hand::subscribeTopicsServices(){
	srv_plan_ = node_handle_.advertiseService("/plan_hand/plan",&plan_hand::callbackPlan,this);
}

bool plan_hand::callbackPlan(planner::plan_req::Request& req, planner::plan_req::Response& res) {
    Vector start = req.start;
    Vector goal = req.goal;

    if (start.size() != n || goal.size() != n) {
        ROS_ERROR("[planner_node] Wrong start and/or goal inputs to planner.");
        return true;   
    }

    res.solved = plan(start, goal, 150); // Planning time: currently 10sec

	return true;
}

void plan_hand::Spin() {
    
    Vector s = {71.8, -410, 70, -70};
    Vector g = {187, -387, 121, -142};

    plan(s, g, 1500);
    
    
    // ros::Rate rate(15);
    // ROS_INFO("[planner node] Initiated planner - waiting for service request...");
    // while (ros::ok()) {

    //     ros::spinOnce();
	// 	rate.sleep();
    // }
}

// -----------------------------------------------------------------------------------------

bool isStateValid(const ob::State *state) {
	return true;
}


ob::PlannerPtr plan_hand::allocatePlanner(ob::SpaceInformationPtr si, plannerType p_type)
{
    switch (p_type)
    {
        case PLANNER_RRT:
        {
            return std::make_shared<og::RRT>(si);
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}


void plan_hand::getPath(ob::ProblemDefinitionPtr pdef, ppMatrix &M) {

	og::PathGeometric Path( dynamic_cast< const og::PathGeometric& >( *pdef->getSolutionPath()));
	const std::vector< ob::State* > &states = Path.getStates();
	ob::State *state;
	Vector q(n);
	for( size_t i = 0 ; i < states.size( ) ; ++i ) {
		state = states[i]->as< ob::State >();
        for (size_t j = 0; j < n; j++)
            q[j] = state->as<ob::RealVectorStateSpace::StateType>()->values[j];
		M.push_back(q);
	}
}

bool plan_hand::plan(Vector q_start, Vector q_goal, double runtime, plannerType p_type) {

    cout << "Initiating planner..." << endl;

	// construct the state space we are planning in
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(n)); 

	ob::RealVectorBounds Qbounds(n);
    // These are the boundaries for Cylinder 25 with feature conf. 5 (obj. position and gripper load)
    Qbounds.setLow(0, -190); // x_min
    Qbounds.setHigh(0, 375); // x_max
    Qbounds.setLow(1, -416); // y_min
    Qbounds.setHigh(1, -126); // y_max
    Qbounds.setLow(2, -14); // load1_min
    Qbounds.setHigh(2, 500); // load1_max
    Qbounds.setLow(3, -516); // load2_min
    Qbounds.setHigh(3, 22); // load2_max

	// set the bound for the space
	Q->as<ob::RealVectorStateSpace>()->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Qspace(Q);

	 // construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Qspace));

	 // set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.03); // 3% ???

	// create a random start state
	ob::ScopedState<ob::RealVectorStateSpace> start(Qspace);
	for (int i = 0; i < n; i++)
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = q_start[i];

	 // create a random goal state
	ob::ScopedState<ob::RealVectorStateSpace> goal(Qspace);
	for (int i = 0; i < n; i++)
		goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = q_goal[i];

	 // create a problem instance
	 ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	 // set the start and goal states
	 pdef->setStartAndGoalStates(start, goal);

	 // create a planner for the defined space
	 // To add a planner, the #include library must be added above
	 ob::PlannerPtr planner = allocatePlanner(si, p_type);

	 // set the problem we are trying to solve for the planner
	 planner->setProblemDefinition(pdef);

	 // perform setup steps for the planner
	 planner->setup();

	 //planner->printSettings(std::cout); // Prints some parameters such as range
	 //planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	 // print the settings for this space
	//  si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	 //si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	 // print the problem settings
	 pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	 // attempt to solve the problem within one second of planning time
	 auto sT = Clock::now();
	 ob::PlannerStatus solved = planner->solve(runtime);
	 double Ttime = std::chrono::duration<double>(Clock::now() - sT).count();

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution in " << Ttime << " seconds." << std::endl;

		Path.clear();
		getPath(pdef, Path);

		// print the path to screen
		path->print(std::cout);  // Print as vectors

		// Save path to file
		// std::ofstream myfile;
		// myfile.open("/home/pracsys/catkin_ws/src/rutgers_collab/src/planner/paths/path.txt");
		// og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		// pog.printAsMatrix(myfile); // Print as matrix to file
		// myfile.close();
	}
	 else
	    std::cout << "No solution found" << std::endl;

    return solved;
}
