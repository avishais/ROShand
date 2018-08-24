/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

//#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

#include "myRRT.hpp"

ompl::geometric::RRT::RRT(const base::SpaceInformationPtr &si)
: base::Planner(si, "RRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRT::setRange, &RRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates, "0,1");

    srv_predict_ = node_handle_.serviceClient<gp_predict::StateAction2State>("/predictWithState");
}

ompl::geometric::RRT::~RRT()
{
    freeMemory();
}

void ompl::geometric::RRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::RRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::RRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::RRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    goalState_.resize(n_);
    retrieveStateVector(pis_.nextGoal(), goalState_);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->action.push_back(0);
        motion->action.push_back(0);
        nn_->add(motion);       
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    base::State *dstate = si_->allocState();

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        si_->copyState(dstate, nmotion->state);

        /* Choose random action */
        Vector a = A[rng_.uniformInt(0,A.size()-1)]; 

        std::cout << "-----\n";
        /* Propagate by predicting next state (GP) */
        predict(dstate, a); // dstate is updated with the new state

        if (1) //si_->checkMotion(nmotion->state, dstate))
        {
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->action.push_back(a[0]);
            motion->action.push_back(a[1]);
            nn_->add(motion);

            printStateVector(motion->parent->state);
            std::cout << nn_->size() << std::endl;

            nmotion = motion;

            double dist = 0.0;
            bool sat = checkSubspaceGoal(motion->state, dist); //goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        save2file(mpath);

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::RRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

void ompl::geometric::RRT::retrieveStateVector(const ob::State *state, Vector &q) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	// Set state of rod
	for (unsigned i = 0; i < n_; i++) {
		q[i] = Q->values[i];
	}
}

void ompl::geometric::RRT::updateStateVector(const ob::State *state, Vector q) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	for (unsigned i = 0; i < n_; i++) {
		Q->values[i] = q[i];
	}
}

void ompl::geometric::RRT::printStateVector(const ob::State *state) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

    std::cout << "q: ["; 
	for (unsigned i = 0; i < n_; i++) 
		std::cout << Q->values[i] << " "; 
    std::cout << "]\n";
	
}

bool ompl::geometric::RRT::checkSubspaceGoal(const ob::State *st, double &dist) {

    Vector state(n_);
    retrieveStateVector(st, state);

    double sum = 0;
    for (int i = 0; i < 2; i++)
        sum += (state[i]-goalState_[i])*(state[i]-goalState_[i]);

    dist = sqrt(sum);

    if (dist < 2)
        return true;
    else
        return false;
}

void ompl::geometric::RRT::predict(ob::State *st, Vector action) {

    Vector cur_state(n_);
    retrieveStateVector(st, cur_state);
    
    gp_predict::StateAction2State srv;
    srv.request.action = action;
    srv.request.state = cur_state;

    srv_predict_.call(srv);

    Vector next_state = srv.response.next_state;

    updateStateVector(st, next_state);
}

void ompl::geometric::RRT::save2file(std::vector<Motion*> mpath) {

    std::cout << "Logging path to files..." << std::endl;

	Vector q(n_);

    // Open a_path file
    std::ofstream pathfile, actionfile;
    pathfile.open("/home/pracsys/catkin_ws/src/rutgers_collab/src/planner/paths/path.txt");
    actionfile.open("/home/pracsys/catkin_ws/src/rutgers_collab/src/planner/paths/actionPath.txt");

    for (int i = mpath.size()-1 ; i >= 0; i--) {
        retrieveStateVector(mpath[i]->state, q);
        for (int j = 0; j < n_; j++) {
            pathfile << q[j] << " ";
        }
        pathfile << std::endl;

        if (i > 0)
            actionfile << mpath[i-1]->action[0] << " " << mpath[i-1]->action[1] << std::endl;
        else
            actionfile << 0 << " " << 0 << std::endl;
    }
    pathfile.close();
    actionfile.close();
}