
# include <ompl/geometric/planners/prm/PRM.h>
# include <ompl/base/SpaceInformation.h>
# include <ompl/base/spaces/RealVectorStateSpace.h>
# include <ompl/geometric/SimpleSetup.h>
# include <ompl/base/ScopedState.h>
# include <ompl/base/spaces/SE3StateSpace.h>
# include <ompl/base/ProblemDefinition.h>
# include <fstream>
# include <iostream>
# include <vector>
# include <ompl/base/PlannerStatus.h>
# include <ompl/geometric/planners/rrt/RRT.h>
# include <ompl/control/SimpleSetup.h>

# include "CollisionChecking.h"
# include "oldRRT.h"
# include "dRRT.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

og::PRM::Graph r1RM;
og::PRM::Graph r2RM;
og::PRM::Graph r3RM;
og::PRM::Graph r4RM;

std::vector<ompl::base::State *> r1RM_nodes;
std::vector<ompl::base::State *> r2RM_nodes;
std::vector<ompl::base::State *> r3RM_nodes;
std::vector<ompl::base::State *> r4RM_nodes;

void createLREnvironment(std::vector<Rectangle> & obstacles){
    Rectangle left;
    left.x = 1;
    left.y = 3;
    left.width = 2;
    left.height = 4;

    Rectangle right;
    right.x = 5;
    right.y = 3;
    right.width = 2;
    right.height = 4;
    obstacles.push_back(left);
    obstacles.push_back(right);
}

// Takes in no arguments as of now, but can modify this later
// Goal is to have this function set up one robot in our environment
og::SimpleSetupPtr createRobot(double goalX, double goalY, double startX, double startY, const std::vector<Rectangle> &obstacles)
{
    auto r2(std::make_shared<ob::RealVectorStateSpace>(2));

    ob::RealVectorBounds r2_bounds(2);
    r2_bounds.setLow(0, 1);
    r2_bounds.setHigh(0, 7);
    r2_bounds.setLow(1, 1);
    r2_bounds.setHigh(1, 9);
    r2->setBounds(r2_bounds);

    og::SimpleSetupPtr ss = std::make_shared<og::SimpleSetup>(r2);
    ob::SpaceInformationPtr si = ss->getSpaceInformation();
    
    ss->setStateValidityChecker(std::bind(isValidStatePoint, std::placeholders::_1, obstacles));

    ob::ScopedState<> start(r2);
    start[0] = startX;
    start[1] = startY;

    ob::ScopedState<> goal(r2);
    goal[0] = goalX;
    goal[1] = goalY;

    // No goal radius for now
    ss->setStartAndGoalStates(start, goal);

    ss->setup();

    return ss;

}

std::vector<ompl::base::State *> createPRMNodes(og::PRM::Graph roadmap){

    // Maps vertices to state - name of property is og::PRM::vertex_state_t() 
    auto stateMap = boost::get(ompl::geometric::PRM::vertex_state_t(), roadmap);

    // Iterate through graph nodes and extract states into vector
    og::PRM::Graph::vertex_iterator v, vend;
    std::vector<ompl::base::State *> states;
    for (boost::tie(v, vend) = vertices(roadmap); v != vend; ++v) {

        ompl::base::State *state = boost::get(stateMap, *v);
        states.push_back(state);
        
        // TODO: comment out, here to ensure/demo that I extracted the x, y, coordinates 
        //double x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        //double y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];

        //std::cout << "x: " << x << std::endl;
        //std::cout << "y: " << y << "\n" << std::endl;
        
    }
    return states;
}

void planRobot(og::SimpleSetupPtr & ss, const char* robotID)
{
    auto prmPtr = std::make_shared<og::PRM>(ss->getSpaceInformation());
    ss->setPlanner(prmPtr);

    //solve the problem:
    ob::PlannerStatus solved = ss->solve(10.0);

    if (solved)
    {
        std::cout << "Found Solution:" << std::endl;
        std::cout << robotID << std::endl;

        auto path = ss->getSolutionPath();
        path.printAsMatrix(std::cout);
        if (robotID == "Robot 1")
        {
            r1RM = prmPtr->getRoadmap();
            r1RM_nodes = createPRMNodes(r1RM);
        }
        if (robotID == "Robot 2")
        {
            r2RM = prmPtr->getRoadmap();
            r2RM_nodes = createPRMNodes(r2RM);
        }
        if (robotID == "Robot 3")
        {
            r3RM = prmPtr->getRoadmap();
            r3RM_nodes = createPRMNodes(r3RM);
        }
        if (robotID == "Robot 4")
        {
            r4RM = prmPtr->getRoadmap();
            r4RM_nodes = createPRMNodes(r4RM);
        }

        // Below creates a plannerData object and stores our full roadmap to it
        // The graphViz is a way to print visualization but isn't in matrix format, want to figure out how to get this so we can better visualize the individual roadmaps
        // TODO: asked for help with ^^^ on piazza, waiting for response :)
        //ob::PlannerData roadMap(ss->getSpaceInformation());
        //ss->getPlannerData(roadMap);
        //roadMap.printGraphviz(std::cout);


    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
    }

    // TODO: might need to clear the planner? 

}


bool collisionForRobotConfigs(ob::State* r1, ob::State* r2, ob::State* r3, ob::State* r4)
{
    // Ensuring no collisions amongst robots for their current positions

    double r1X = r1->as<ob::RealVectorStateSpace::StateType>()->values[0];
    double r1Y = r1->as<ob::RealVectorStateSpace::StateType>()->values[1];

    double r2X = r2->as<ob::RealVectorStateSpace::StateType>()->values[0];
    double r2Y = r2->as<ob::RealVectorStateSpace::StateType>()->values[1];

    double r3X = r3->as<ob::RealVectorStateSpace::StateType>()->values[0];
    double r3Y = r3->as<ob::RealVectorStateSpace::StateType>()->values[1];

    double r4X = r4->as<ob::RealVectorStateSpace::StateType>()->values[0];
    double r4Y = r4->as<ob::RealVectorStateSpace::StateType>()->values[1];

    // Probably less messy way of doing this, but leaving for now:
    if(((r1X == r2X) and (r1Y == r2Y)) or ((r1X == r3X) and (r1Y == r3Y)) or ((r1X == r4X) and (r1Y == r4Y)))
    {
        return false;
    }
    if(((r2X == r3X) and (r2Y == r3Y)) or ((r2X == r4X) and (r2Y == r4Y)))
    {
        return false;
    }
    if(((r3X == r4X) and (r3Y == r4Y)))
    {
        return false;
    }

    // No collisions in position, valid configuration to add:
    return true;

}

std::vector<std::vector<ob::ScopedState<>>> createTensorRM(ob::StateSpacePtr r1Space, ob::StateSpacePtr r2Space, ob::StateSpacePtr r3Space, ob::StateSpacePtr r4Space)
{

    // New function creating the tensor product composite states vector (now don't have to worry about PRM size)
    std::vector<std::vector<ob::ScopedState<>>> compStates;

    for(size_t i1 = 0; i1 < r1RM_nodes.size(); i1++)
    {
        for(size_t i2 = 0; i2 < r2RM_nodes.size(); i2++)
        {
            for(size_t i3 = 0; i3 < r3RM_nodes.size(); i3++)
            {
                for(size_t i4 = 0; i4 < r4RM_nodes.size(); i4++)
                {
                    bool noCollision = collisionForRobotConfigs(r1RM_nodes[i1], r2RM_nodes[i2], r3RM_nodes[i3], r4RM_nodes[i4]);
                    if(noCollision)
                    {
                        // Now create an 'empty' vector to add to compStates:
                        std::vector<ob::ScopedState<>> tempVec;

                        // Add each config to it:
                        tempVec.push_back(ob::ScopedState<>(r1Space, r1RM_nodes[i1]));
                        tempVec.push_back(ob::ScopedState<>(r2Space, r2RM_nodes[i2]));
                        tempVec.push_back(ob::ScopedState<>(r3Space, r3RM_nodes[i3]));
                        tempVec.push_back(ob::ScopedState<>(r4Space, r4RM_nodes[i4]));

                        // Add full vector to the compState one:
                        compStates.push_back(tempVec);
                    }
                }
            }
        }
    }

    return compStates;
}


bool isStateValid(const ob::SpaceInformationPtr si, const ob::State * state, std::vector<Rectangle> & obstacles)
{
    const ob::CompoundState * cstate = state->as<ob::CompoundState>();

    double r1X = cstate->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double r1Y = cstate->as<ob::RealVectorStateSpace::StateType>(0)->values[1];

    double r2X = cstate->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
    double r2Y = cstate->as<ob::RealVectorStateSpace::StateType>(1)->values[1];

    double r3X = cstate->as<ob::RealVectorStateSpace::StateType>(2)->values[0];
    double r3Y = cstate->as<ob::RealVectorStateSpace::StateType>(2)->values[1];

    double r4X = cstate->as<ob::RealVectorStateSpace::StateType>(3)->values[0];
    double r4Y = cstate->as<ob::RealVectorStateSpace::StateType>(3)->values[1];
    
    // No collisions in position, valid configuration to add:
    return si->satisfiesBounds(state) && isValidPoint(r1X, r1Y, obstacles) && isValidPoint(r2X, r2Y, obstacles) && isValidPoint(r3X, r3Y, obstacles) && isValidPoint(r4X, r4Y, obstacles);
}

void compositeSolve(og::SimpleSetupPtr r1, og::SimpleSetupPtr r2, og::SimpleSetupPtr r3, og::SimpleSetupPtr r4, 
double r1SX, double r1SY, double r1GX, double r1GY, double r2SX, double r2SY, double r2GX, double r2GY, 
double r3SX, double r3SY, double r3GX, double r3GY, double r4SX, double r4SY, double r4GX, double r4GY, std::vector<Rectangle> obstacles)
{
    std::vector<std::vector<ob::ScopedState<>>> compositeState = createTensorRM(r1->getStateSpace(), r2->getStateSpace(), r3->getStateSpace(), r4->getStateSpace());


    auto stateSpace = r1->getStateSpace() + r2->getStateSpace() + r3->getStateSpace() + r4->getStateSpace();

    for (size_t i = 0; i < compositeState.size(); ++i) {
        ob::CompoundStateSpace * compStateSpace = stateSpace->as<ob::CompoundStateSpace>();     // cast to compound state space
        ob::CompoundState *compState = compStateSpace->allocState()->as<ob::CompoundState>();   // allocate space for compound state
        for (size_t j = 0; j < compositeState[i].size(); ++j) {
            ob::ScopedState<> tempState = compositeState[i][j];                                 // extract state from matrix of states
            compStateSpace->getSubspace(j)->copyState(compState->as<ob::State>(j), tempState.get());    // copy over the state as a substate of a composite state
        }
    }

    og::SimpleSetupPtr compound = std::make_shared<og::SimpleSetup>(ob::StateSpacePtr(stateSpace));

    // Set the start state using the coordinates set in the lines above
    ob::ScopedState<ob::CompoundStateSpace> start(compound->getSpaceInformation());
    start->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = r1SX;
    start->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = r1SY;

    start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = r2SX;
    start->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = r2SY;

    start->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = r3SX;
    start->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = r3SY;

    start->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = r4SX;
    start->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = r4SY;

    // Set the goal state using the coordinates set in the lines above
    ob::ScopedState<ob::CompoundStateSpace> goal(compound->getSpaceInformation());
    goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = r1GX;
    goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = r1GY;

    goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = r2GX;
    goal->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = r2GY;

    goal->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = r3GX;
    goal->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = r3GY;

    goal->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = r4GX;
    goal->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = r4GY;

    compound->setStartAndGoalStates(start, goal);


    // Sets state validity checker but has no effect on running the code for some reason
    ob::SpaceInformationPtr si = compound->getSpaceInformation();
    compound->setStateValidityChecker(
        [si, &obstacles](const ob::State* state) { return isStateValid(si, state, obstacles); }
    );
    compound->setPlanner(std::make_shared<ompl::geometric::RRT>(compound->getSpaceInformation()));

    compound->setup();

    ob::PlannerStatus solved = compound->solve(20);

    if (solved)
    {
        std::cout << "Found Solution:" << std::endl;

        auto path = compound->getSolutionPath();
        path.printAsMatrix(std::cout);
    }
    else{
        std::cout << "No Solution Found" << std::endl;
    }
}

ob::StateSamplerPtr customStateSampler(const ob::StateSpacePtr space)
{

}

int main(int, char **)
{
    std::vector<Rectangle> obstacles;
    createLREnvironment(obstacles);
    
    // Creating the varius robot start and goal states: 
    // Right now, doing the left/right swap from example, since just PRM right now, pretty simple solutions

    double r1SX = 2.0;
    double r1SY = 8.0;
    double r1GX = 6.0;
    double r1GY = 8.0;

    double r2SX = 6.0;
    double r2SY = 8.0;
    double r2GX = 2.0;
    double r2GY = 8.0;

    double r3SX = 2.0;
    double r3SY = 2.0;
    double r3GX = 6.0;
    double r3GY = 2.0;

    double r4SX = 6.0;
    double r4SY = 2.0;
    double r4GX = 2.0;
    double r4GY = 2.0;

    og::SimpleSetupPtr r1 = createRobot(r1GX, r1GY, r1SX, r1SY, obstacles);
    og::SimpleSetupPtr r2 = createRobot(r2GX, r2GY, r2SX, r2SY, obstacles);
    og::SimpleSetupPtr r3 = createRobot(r3GX, r3GY, r3SX, r3SY, obstacles);
    og::SimpleSetupPtr r4 = createRobot(r4GX, r4GY, r4SX, r4SY, obstacles);

    planRobot(r1, "Robot 1");
    planRobot(r2, "Robot 2");
    planRobot(r3, "Robot 3");
    planRobot(r4, "Robot 4");

    // Having composite RM return a vector of vectors of states so that we can sample from those in dRRT
    // Update, changed to tensor product, eliminates alot of our sizing issues with the states yippee!! -> pey :)

    // Creates and solve a composite TENSOR roadmap of the 4 robots (naive approach)
    // compositeSolve(r1, r2, r3, r4, r1SX, r1SY, r1GX, r1GY, r2SX, r2SY, r2GX, r2GY, r3SX, r3SY, r3GX, r3GY, r4SX, r4SY, r4GX, r4GY, obstacles);

    // Now, we want to plan with our implicit search dRRT, using our individual PRM roadmaps (not doing tensor product)

    // Make a compound state space with the spaces of our 4 robot PRMS:
    auto stateSpace = r1->getStateSpace() + r2->getStateSpace() + r3->getStateSpace() + r4->getStateSpace();

    // Initialize a simple setup pointer:
    oc::SimpleSetupPtr compound = std::make_shared<oc::SimpleSetup>(ob::StateSpacePtr(stateSpace));

    // Since we have our start and goal states, set these:

    ob::ScopedState<ob::CompoundStateSpace> start(compound->getSpaceInformation());
    start->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = r1SX;
    start->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = r1SY;

    start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = r2SX;
    start->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = r2SY;

    start->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = r3SX;
    start->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = r3SY;

    start->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = r4SX;
    start->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = r4SY;

    ob::ScopedState<ob::CompoundStateSpace> goal(compound->getSpaceInformation());
    goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = r1GX;
    goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = r1GY;

    goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = r2GX;
    goal->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = r2GY;

    goal->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = r3GX;
    goal->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = r3GY;

    goal->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = r4GX;
    goal->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = r4GY;

    compound->setStartAndGoalStates(start, goal);

    // TODO: skipping validity checker setup for now because I think that is something more related to dRRT?

    // Set our planner as dRRT
    compound->setPlanner(std::make_shared<ompl::control::dRRT>(compound->getSpaceInformation()));
    
    // Trying to set the members holding vector of states for each robot by accessing our planner (dRRT) and calling setRobotNodes
    // Not sure if this will work? 

    // TODO: Does not work lol, need to investigate this!!
    compound->getPlanner()->setRobotNodes(r1RM_nodes, r2RM_nodes, r3RM_nodes, r4RM_nodes);
    
    compound->setup();


    // Solve with dRRT (obviously not working yet)
    ob::PlannerStatus solved = compound->solve(20);

    if (solved)
    {
        std::cout << "Found Solution:" << std::endl;

        auto path = compound->getSolutionPath();
        path.printAsMatrix(std::cout);
    }
    else{
        std::cout << "No Solution Found" << std::endl;
    }

}