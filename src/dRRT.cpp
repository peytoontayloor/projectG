/* Author: Ioan Sucan */

#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <vector>
#include <utility>
#include <ompl/base/goals/GoalState.h>

#include "dRRT.h"

ompl::geometric::dRRT::dRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "dRRTintermediate" : "dRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &dRRT::setRange, &dRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &dRRT::setGoalBias, &dRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &dRRT::setIntermediateStates, &dRRT::getIntermediateStates,
                                "0,1");
    
    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::dRRT::~dRRT()
{
    freeMemory();
}

void ompl::geometric::dRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::dRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::dRRT::freeMemory()
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

ompl::base::State * ompl::geometric::dRRT::customCompositeSampler(ompl::base::StateSpacePtr space, ompl::base::State *goal)
{

    // ALSO! Need to collision check against obstacles here? Or somewhere- Not sure? 

    // Sample a random state from each vector uniformly

    int i1 = rng_.uniformInt(0, robot1.size() - 1);
    int i2 = rng_.uniformInt(0, robot2.size() - 1);
    int i3 = rng_.uniformInt(0, robot3.size() - 1);
    int i4 = rng_.uniformInt(0, robot4.size() - 1);

    // Create a state pointer to hold each of the randomly sampled states
    ompl::base::State* r1State = robot1[i1];
    ompl::base::State* r2State = robot2[i2];
    ompl::base::State* r3State = robot3[i3];
    ompl::base::State* r4State = robot4[i4];

    // TODO: Need to make sure none of our states are in the goal states corresponding dimension
    // If r1 is equal to dimension 1 goal state, then exit null and resample, and so on
    /*ompl::base::State *goal1 = goal->as<ompl::base::CompoundState>()->components[0];
    double x1 = goal1->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double y1 = goal1->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    if((x1 == r1State->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]) and (y1 == r1State->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]))
    {
        return nullptr;
    }

    ompl::base::State *goal2 = goal->as<ompl::base::CompoundState>()->components[1];
    double x2 = goal2->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double y2 = goal2->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    if((x2== r2State->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]) and (y2 == r2State->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]))
    {
        return nullptr;
    }

    ompl::base::State *goal3 = goal->as<ompl::base::CompoundState>()->components[2];
    double x3 = goal3->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double y3 = goal3->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    if((x3 == r3State->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]) and (y3== r3State->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]))
    {
        return nullptr;
    }

    ompl::base::State *goal4 = goal->as<ompl::base::CompoundState>()->components[3];
    double x4 = goal4->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double y4 = goal4->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    if((x4 == r4State->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]) and (y4 == r4State->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]))
    {
        return nullptr;
    }*/

    // Need to collision check here!
                
    // Casting as a compound state to add r1, r2, r3, and r4 to it
    ompl::base::CompoundStateSpace * compound = space->as<ompl::base::CompoundStateSpace>();
    
    // Initializing our state to return
    ompl::base::State* returnState = compound->allocState();

    // In order to use components to set the subspaces, need to cast our return state as a compound state
    ompl::base::CompoundState* cmp = returnState->as<ompl::base::CompoundState>();

    // Looking at OMPL documentation, I think we can use "components" to acess each substate --> yes
    // Can now set each subspace/state in our compound state to r1->r4
    compound->getSubspace(0)->copyState(cmp->components[0], r1State);
    compound->getSubspace(1)->copyState(cmp->components[1], r2State);
    compound->getSubspace(2)->copyState(cmp->components[2], r3State);
    compound->getSubspace(3)->copyState(cmp->components[3], r4State);

    // TODO: need to check that returnState is not in explored set! 
    for(size_t i = 0; i < explored.size(); i++)
    {
        if (returnState == explored[i])
        {
            return nullptr;
        }
    }
    return returnState;

}


ompl::base::PlannerStatus ompl::geometric::dRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    if (!goal_s->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Leaving for now, allocating but not working with is (defining our own sampling method)
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    //base::State *xstate = si_->allocState();
    auto *g = goal->as<ompl::base::GoalState>();
   // ompl::base::State *gState = g->getState();
    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if (rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
        {
            // TODO: want rstate to be a compound state formed from picking a random configuration from our 4 robot's PRMS
            // sampler_->sampleUniform(rstate);
            //ompl::base::CompoundStateSampler compoundStateSampler_ (si_->getStateSpace().get());
            //compoundStateSampler_.sampleUniform(rstate);
            
            //rstate = getCompositeStates(si_->getStateSpace(), si_);
            ompl::base::State* tempState = customCompositeSampler(si_->getStateSpace(), g->getState());
            if (tempState == nullptr)
            {
                //si_->freeState(tempState);
                continue;
            }
            si_->copyState(rstate, tempState);
            si_->freeState(tempState);

        }
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        //base::State *dstate = rstate;

        // nmotion->state is our qnear! 
        // First, find k-nearest neighbors of qnear (right now, k is 2)

        std::vector<ompl::base::State *> nbrR1 = neighbors(nmotion->state, 1);
        std::vector<ompl::base::State *> nbrR2 = neighbors(nmotion->state, 2);
        std::vector<ompl::base::State *> nbrR3 = neighbors(nmotion->state, 3);
        std::vector<ompl::base::State *> nbrR4 = neighbors(nmotion->state, 4);

        if ((nbrR1.empty()) || (nbrR2.empty()) || (nbrR3.empty()) || (nbrR4.empty()))
        {
            std::cout << "NO NEIGHBOR, PICKING NEW QRAND" << std::endl;
            continue;
        }

        // Now, need a qnew from each robots graph according to each robots qnear:
        // Extract individual state for each robot from qnear AND rstate
        // Need to pass each robot's individual state information pointers in, not the main compound one
        ompl::base::State* qNew1 = oracle(nmotion->state, rstate, nbrR1, 1, spaceInfo1);
        ompl::base::State* qNew2 = oracle(nmotion->state, rstate, nbrR2, 2, spaceInfo2);
        ompl::base::State* qNew3 = oracle(nmotion->state, rstate, nbrR3, 3, spaceInfo3);
        ompl::base::State* qNew4 = oracle(nmotion->state, rstate, nbrR4, 4, spaceInfo4);

        // For debugging seg faults
        // std::cout << "Line 277" << std::endl;
        // Given all of the individual qnews, make our composite QNEW:
        ompl::base::CompoundStateSpace* compound = (si_->getStateSpace())->as<ompl::base::CompoundStateSpace>();
        // std::cout << "Line 282" << std::endl;
        ompl::base::State* qNew = compound->allocState();
        // std::cout << "Line 284" << std::endl;
        ompl::base::CompoundState* qNewCompound = qNew->as<ompl::base::CompoundState>();
        // std::cout << "Line 286" << std::endl;

        // UPDATE: getting segault bc qnew1, qnew2, and qnew3 are null pointers (means oracle function is behind this)
        if(qNew1 == nullptr)
        {
            std::cout << "new 1" << std::endl;
            continue;
        }
        if(qNew2 == nullptr)
        {
            std::cout << "new 2" << std::endl;
            continue;
        }
        if(qNew3 == nullptr)
        {
            std::cout << "new 3" << std::endl;
            continue;
        }
        if(qNew4 == nullptr)
        {
            std::cout << "new 4" << std::endl;
            continue;
        }
        compound->getSubspace(0)->copyState(qNewCompound->components[0], qNew1);
        compound->getSubspace(1)->copyState(qNewCompound->components[1], qNew2);
        compound->getSubspace(2)->copyState(qNewCompound->components[2], qNew3);
        compound->getSubspace(3)->copyState(qNewCompound->components[3], qNew4);

        // Only work with qnew if it is unexplored 
        bool unexplored = true;
        for(size_t i = 0; i < explored.size(); i++)
        {
            if (qNew == explored[i])
                {
                    unexplored = false;
                    break;
                }
        }
        if (unexplored == false)
        {
            si_->freeState(qNew);
            continue;
        }
        // std::cout << "Line 291" << std::endl;

        // TODO: now need to collision check/local connector! 
        // If everything works out, add qnew to the tree and 'explored' vector- haven't incorporated yet

        // If connector is true, no collision, add qnew to the tree, otherwise continue looping and get new one (I think?)
        std::set<std::pair<double, double>> stay = localConnector(nmotion->state, qNew);
        if (stay.size() > 0){ // if we found a cycle
            ompl::base::State *qnear;
            ompl::base::State *qnew;
            for (int i = 0 ; i < 4; ++i){

                qnear = nmotion->state->as<ompl::base::CompoundState>()->components[i];
                qnew = qNew->as<ompl::base::CompoundState>()->components[i];

                double nearX = qnear->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double nearY = qnear->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                double newX = qnew->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                double newY = qnew->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

                std::pair<double, double> newCoord (newX, newY);
                std::pair<double, double> nearCoord (nearX, nearY);

                if (stay.find(newCoord) != stay.end()) // if robot needs to stay put, set q new to q near
                {
                    qnew->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = qnear->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    qnew->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = qnear->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                }

            }
        }

        // Now add qnew to the tree

        // std::cout << "TRUE" << std::endl;

        auto *motion = new Motion(si_);
        si_->copyState(motion->state, qNew);
        motion->parent = nmotion;
        nn_->add(motion);

        // Insert each robot's state into respective robot's set of explored nodes

        /*double x_new_1 = qNewCompound->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
        double y_new_1 = qNewCompound->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
        std::pair<double, double> explored_1 (x_new_1, y_new_1);
        exploredR1.insert(explored_1);

        double x_new_2 = qNewCompound->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0];
        double y_new_2 = qNewCompound->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1];
        std::pair<double, double> explored_2 (x_new_2, y_new_2);
        exploredR1.insert(explored_2);

        double x_new_3 = qNewCompound->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0];
        double y_new_3 = qNewCompound->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1];  
        std::pair<double, double> explored_3 (x_new_3, y_new_3);
        exploredR1.insert(explored_3);
            
        double x_new_4 = qNewCompound->as<ompl::base::RealVectorStateSpace::StateType>(3)->values[0];
        double y_new_4 = qNewCompound->as<ompl::base::RealVectorStateSpace::StateType>(3)->values[1];
        std::pair<double, double> explored_4 (x_new_4, y_new_4);
        exploredR1.insert(explored_4);*/

        // Add qnew to set of our explored tree:
        explored.push_back(qNew);

        nmotion = motion;

        double dist = 0.0;
        bool sat = goal->isSatisfied(nmotion->state, &dist);
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

    std::cout << "done with loop" << std::endl;

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

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    //si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::geometric::dRRT::getPlannerData(base::PlannerData &data) const
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