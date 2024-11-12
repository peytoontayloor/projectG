/* Author: Ioan Sucan */

#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <vector>
#include <utility>

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

ompl::base::State * ompl::geometric::dRRT::customCompositeSampler(ompl::base::StateSpacePtr space)
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

    return returnState;

}

// ompl::base::State* ompl::geometric::dRRT::smallestDist(ompl::base::State* source, std::vector<ompl::base::State *> robotStates)
// {
//     // To keep track of minimum distance
//     ompl::base::State* minState1;
//     ompl::base::State* minState2;

//     // TODO: change this to either the first two distances in the array, or some other metric that is big enough that logic will still work
//     double minDist1 = 10000000;
//     double minDist2 = 10000000;

//     // Extract source position:
//     // TODO: pretty sure extracting position wrong lol
//     double x = source->position[0];
//     double y = source->position[1];

//     for(size_t i = 0; i < robotStates.size())
//     {
//         double tempX = robotStates[i]->position[0];
//         double tempY = robotStates[i]->position[1];

//         double dist = euclideanDistance(x, double y, double tempX, double tempY)

//         if (dist < minDist1)
//         {
//             minDist1 = dist;

//             // TODO: doing the bad memory management thing again -> but copystate() won't work unless we get r1's specific state information pointer
//             // ^^might be able to use si_ from solve() but nervous I am doing state spaces wrong?
//             minState1 = robotState[i];

//         }
//         else if (dist < minDist2)
//         {
//             minDist2 = dist;
//             minState2 = robotState[i];
//         }
//     }

//     // TODO: not sure how to do tuples in C++, maybe just return a vector (change return type here and in .h file)
//     return minState1, minState2;
// }

// std::vector<ompl::base::State *> ompl::geometric::dRRT::nearestN(ompl::base::State* qnear)
// {
//     // Find smallest distance states from qnear to r1, r2, r3, r4, then make as a composite state and save.
//     // TODO: we need to make sure that the smallest distance state isn't just itself!

//     ompl::base::State* r1Nearest1 = smallestDist(qnear, robot1)[0];
//     ompl::base::State* r2Nearest1 = smallestDist(qnear, robot2)[0];
//     ompl::base::State* r3Nearest1 = smallestDist(qnear, robot3)[0];
//     ompl::base::State* r4Nearest1 = smallestDist(qnear, robot4)[0];

//     ompl::base::State* r1Nearest2 = smallestDist(qnear, robot1)[1];
//     ompl::base::State* r2Nearest2 = smallestDist(qnear, robot2)[1];
//     ompl::base::State* r3Nearest2 = smallestDist(qnear, robot3)[1];
//     ompl::base::State* r4Nearest2 = smallestDist(qnear, robot4)[1];

//     // Create a compound state (same as in custom sampler for qrand)
//     ompl::base::CompoundStateSpace * compound = space->as<ompl::base::CompoundStateSpace>();
//     ompl::base::State* returnState1 = compound->allocState();
//     ompl::base::State* returnState2 = compound->allocState();
//     ompl::base::CompoundState* cmp1 = returnState1->as<ompl::base::CompoundState>();
//     ompl::base::CompoundState* cmp2 = returnState2->as<ompl::base::CompoundState>();

//     compound->getSubspace(0)->copyState(cmp1->components[0], r1Nearest1);
//     compound->getSubspace(1)->copyState(cmp1->components[1], r2Nearest1);
//     compound->getSubspace(2)->copyState(cmp1->components[2], r3Nearest1);
//     compound->getSubspace(3)->copyState(cmp1->components[3], r4Nearest1);

//     compound->getSubspace(0)->copyState(cmp2->components[0], r1Nearest2);
//     compound->getSubspace(1)->copyState(cmp2->components[1], r2Nearest2);
//     compound->getSubspace(2)->copyState(cmp2->components[2], r3Nearest2);
//     compound->getSubspace(3)->copyState(cmp2->components[3], r4Nearest2);

//     // TODO: right now just returning k=1 nearest neighbor, we want at least k=2 and then store them in a vector of states and return!
//     // ^^^ Have k=2 in function but haven't added to a vector yet
//     return returnState1;

// }


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
    base::State *xstate = si_->allocState();

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
            ompl::base::State* tempState = customCompositeSampler(si_->getStateSpace());
            si_->copyState(rstate, tempState);
            si_->freeState(tempState);

        }
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        // nmotion->state is our qnear! 
        // First, find k-nearest neighbors of qnear (right now, k is 2)

        std::vector<ompl::base::State *> nbrR1 = neighbors(nmotion->state, 1);
        std::vector<ompl::base::State *> nbrR2 = neighbors(nmotion->state, 2);
        std::vector<ompl::base::State *> nbrR3 = neighbors(nmotion->state, 3);
        std::vector<ompl::base::State *> nbrR4 = neighbors(nmotion->state, 4);

        if ((nbrR1.empty()) or (nbrR2.empty()) or (nbrR3.empty()) or (nbrR4.empty()))
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
        }
        if(qNew2 == nullptr)
        {
            std::cout << "new 2" << std::endl;
        }
        if(qNew3 == nullptr)
        {
            std::cout << "new 3" << std::endl;
        }
        if(qNew4 == nullptr)
        {
            std::cout << "new 4" << std::endl;
        }
        compound->getSubspace(0)->copyState(qNewCompound->components[0], qNew1);
        compound->getSubspace(1)->copyState(qNewCompound->components[1], qNew2);
        compound->getSubspace(2)->copyState(qNewCompound->components[2], qNew3);
        compound->getSubspace(3)->copyState(qNewCompound->components[3], qNew4);
        // std::cout << "Line 291" << std::endl;

        // TODO: now need to collision check/local connector! 
        // If everything works out, add qnew to the tree and 'explored' vector- haven't incorporated yet

        ompl::base::CompoundState * qNearCompound = nmotion->state->as<ompl::base::CompoundState>();
        std::pair<ompl::base::State *, ompl::base::State *> r1movement (qNearCompound->components[0], qNew1);
        std::pair<ompl::base::State *, ompl::base::State *> r2movement (qNearCompound->components[1], qNew2);
        std::pair<ompl::base::State *, ompl::base::State *> r3movement (qNearCompound->components[2], qNew3);
        std::pair<ompl::base::State *, ompl::base::State *> r4movement (qNearCompound->components[3], qNew4);

        // point-path collision checker -- may not be needed
        std::vector<std::pair<ompl::base::State *, ompl::base::State *>> robotMovements = {r1movement, r2movement, r3movement, r4movement};
        std::vector<int> robotCollisions = robotPathCollisionChecking(robotMovements);
        // for (size_t i = 0; i < robotCollisions.size(); ++i){
        //     std::cout << robotCollisions[i] << std::endl;
        // }

        // local connector
        addVertex(qNew);
        addVertex(nmotion->state);
        if (vertexInGraph(nmotion->state)){
            std::vector<std::vector<std::pair<double, double>>> nodes = adjList.find(createCoordinates(nmotion->state))->second;
            nodes.push_back(createCoordinates(qNew));
        }
        // TODO: not sure how to do the default dictionary thing where we just add a key into a dictionary
        setIndegrees();
        graphSearch();

        


        double d = customDistanceFunction(nmotion->state, rstate);
        //double d = si_->distance(nmotion->state, rstate);

        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            if (addIntermediateStates_)
            {
                std::vector<base::State *> states;
                const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

                if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
                    si_->freeState(states[0]);

                for (std::size_t i = 1; i < states.size(); ++i)
                {
                    auto *motion = new Motion;
                    motion->state = states[i];
                    motion->parent = nmotion;
                    nn_->add(motion);

                    nmotion = motion;
                }
            }
            else
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;
                nn_->add(motion);

                nmotion = motion;
            }

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