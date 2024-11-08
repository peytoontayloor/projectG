/* Author: Ioan Sucan */

#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>

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

ompl::base::State * ompl::geometric::dRRT::getCompositeStates(ompl::base::StateSpacePtr space)
{
    // Sample a random state from each vector uniformly 
    int i1 = rng_.uniformInt(0, robot1.size() - 1);
    int i2 = rng_.uniformInt(0, robot2.size() - 1);
    int i3 = rng_.uniformInt(0, robot3.size() - 1);
    int i4 = rng_.uniformInt(0, robot4.size() - 1);

    ompl::base::State * r1State = robot1[i1];
    ompl::base::State * r2State = robot2[i2];
    ompl::base::State * r3State = robot3[i3];
    ompl::base::State * r4State = robot4[i4];
                
    // Casting as a compound state to add r1, r2, r3, and r4 to it
    ompl::base::CompoundStateSpace * compound = space->as<ompl::base::CompoundStateSpace>();

    // Initializing our state to return
    ompl::base::State * returnState = compound->allocState();
                
    // // Copying over each of our states as elements of this compound state:
    compound->getSubspace(0)->copyState(returnState, r1State);   
    compound->getSubspace(1)->copyState(returnState, r2State);   
    compound->getSubspace(2)->copyState(returnState, r3State);   
    compound->getSubspace(3)->copyState(returnState, r4State);  

    // compound->as<ompl::base::State>(1)->copyState(r1State); 
    // compound->as<ompl::base::State>(1)->copyState(r2State);
    // compound->as<ompl::base::State>(2)->copyState(r3State);
    // compound->as<ompl::base::State>(3)->copyState(r4State);

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
            ompl::base::CompoundStateSampler compoundStateSampler_ (si_->getStateSpace().get());
            compoundStateSampler_.sampleUniform(rstate);
            // rstate = getCompositeStates(si_->getStateSpace());
            // sampler_->addSampler()

        }
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* TODO: temporarily commenting out oracle section until we fix everything else

        // find q new by minimizing angle between line from q_near--q_rand and q_new---q_rand
        std::vector<Motion *> randnbrs;
        std::size_t k = 5;
        nn_->nearestK(nmotion, k, randnbrs); // TODO: needs to be UNEXPLORED neighbors
        Motion *newmotion;
        double angle = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < randnbrs.size(); i++){
            Motion * tempnew_motion = randnbrs[i];
            double d = distanceFunction(nmotion, tempnew_motion);
            double n = distanceFunction(rmotion, tempnew_motion);
            double m = distanceFunction(nmotion, rmotion); 
            double numerator = d * d + m * m - n * n;
            double denominator = 2 * d * m;
            double temp_angle = acos(numerator / denominator);
            if (temp_angle < angle){
                newmotion = tempnew_motion;
            }
        }

        */

        // TODO: SEGFAULT HAPPENING HERE 
        // Sometimes (rarely) there is no segfault, but when there is I am pretty sure it happens here (line 183)
        double d = customDistanceFunction(nmotion->state, rstate);

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