/* Author: Ioan Sucan */

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/base/PlannerStatus.h"
#include "ompl/base/Planner.h"
#include "ompl/base/Goal.h"
#include "ompl/base/State.h"
#include <limits>
#include <math.h>
#include <cmath>

#include "RRT.h"

ompl::control::dRRT::dRRT(const SpaceInformationPtr &si) : ompl::base::Planner(si, "dRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &dRRT::setGoalBias, &dRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &dRRT::setIntermediateStates, &dRRT::getIntermediateStates,
                                "0,1");
}

ompl::control::dRRT::~dRRT()
{
    freeMemory();
}

void ompl::control::dRRT::setup()
{
    ompl::base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::control::dRRT::clear()
{
    ompl::base::Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::dRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

ompl::base::State getCompositeStates(std::vector<ompl::base::State *> r1, std::vector<ompl::base::State *> r2, std::vector<ompl::base::State *> r3, std::vector<ompl::base::State *> r4, ompl::base::StateSpacePtr space)
{
    // Sample a random state from each vector uniformly 
    size_t i1 = rng_.uniformInt(0, r1.size() - 1);
    size_t i2 = rng_.uniformInt(0, r2.size() - 1);
    size_t i3 = rng_.uniformInt(0, r3.size() - 1);
    size_t i4 = rng_.uniformInt(0, r4.size() - 1);

    ompl::base::State r1State = r1[i1];
    ompl::base::State r2State = r2[i2];
    ompl::base::State r3State = r3[i3];
    ompl::base::State r4State = r4[i4];
                
    // Initializing our state to return
    ompl::base::State returnState = space.allocState();

    // Casting as a compound state to add r1, r2, r3, and r4 to it
    ompl::base::CompoundState compound = returnState->as<ompl::base::CompoundState>();
                
    // Copying over each of our states as elements of this compound state:
    compound->as<ompl::base::State>(0)->copyState(r1State);
    compound->as<ompl::base::State>(1)->copyState(r2State);
    compound->as<ompl::base::State>(2)->copyState(r3State);
    compound->as<ompl::base::State>(3)->copyState(r4State);

    return returnState;

}

ompl::base::PlannerStatus ompl::control::dRRT::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    ompl::base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);

    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const ompl::base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }

    if (!goal_s->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_GOAL;
    }

    // TODO: trying to change original RRT code: sampler_ = si_->allocStateSampler(); to sampling from a custom sampler
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_); // qrand

    //UPDATE --> our rstate needs to be of type compound
    //ompl::base::State *rstate = rmotion->state;
    ompl::base::CompoundState *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    ompl::base::State *xstate = si_->allocState();

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if (rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            // TODO: want rstate to be a compound state formed from picking a random configuration from our 4 robot's PRMS
            //sampler_->sampleUniform(rstate);
            rstate = getCompositeStates(robot1, robot2, robot3, robot4, si_->getStateSpace());

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion); // qnear

        /* find q new by minimizing angle between line from q_near--q_rand and q_new---q_rand*/
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

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

        if (addIntermediateStates_)
        {
            // this code is contributed by Jennifer Barry
            std::vector<ompl::base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for (; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    auto *motion = new Motion();
                    motion->state = pstates[p];
                    // we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    nn_->add(motion);
                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }

                // free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (auto &pstate : pstates)
                    si_->freeState(pstate);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                /* create a motion */
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;

                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
                {
                    approxdif = dist;
                    solution = motion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
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
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::control::dRRT::getPlannerData(base::PlannerData &data) const
{
    ompl::base::Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(ompl::base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(ompl::base::PlannerDataVertex(m->parent->state), ompl::base::PlannerDataVertex(m->state),
                             ompl::control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(ompl::base::PlannerDataVertex(m->parent->state), ompl::base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(ompl::base::PlannerDataVertex(m->state));
    }
}