
# include <ompl/geometric/planners/prm/PRM.h>
# include <ompl/base/SpaceInformation.h>
# include <ompl/base/spaces/RealVectorStateSpace.h>
# include <ompl/control/spaces/RealVectorControlSpace.h>
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
#include <ompl/control/ODESolver.h>
#include <string>

#include <ompl/geometric/SimpleSetup.h>

# include "CollisionChecking.h"
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


// Takes in no arguments as of now, but can modify this later
// Goal is to have this function set up one robot in our environment
og::SimpleSetupPtr createClockRobot(double goalX, double goalY, double startX, double startY, const std::vector<Rectangle> &obstacles)
{
    auto r2(std::make_shared<ob::RealVectorStateSpace>(2));

    ob::RealVectorBounds r2_bounds(2);
    r2_bounds.setLow(0, 0);
    r2_bounds.setHigh(0, 11);
    r2_bounds.setLow(1, 0);
    r2_bounds.setHigh(1, 11);
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

    // An attempt to cut down on the neighbors we are searching through in dRRT
    //prmPtr->setMaxNearestNeighbors(10);

    //solve the problem:
    ob::PlannerStatus solved = ss->solve(30.0);

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
    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
    }

    // TODO: might need to clear the planner? 
}


void planClockRobots(std::vector<og::SimpleSetupPtr> & ssVector, std::vector<og::PRM::Graph> & roadmaps)
{
    for (size_t i = 0; i < ssVector.size(); ++i)
    {
        auto prmPtr = std::make_shared<og::PRM>(ssVector[i]->getSpaceInformation());
        ssVector[i]->setPlanner(prmPtr);

        ob::PlannerStatus solved = ssVector[i]->solve(30.0);

        if (solved)
        {
            std::cout << "Found Solution:" << std::endl;

            auto path = ssVector[i]->getSolutionPath();
            path.printAsMatrix(std::cout);
            og::PRM::Graph rm = prmPtr->getRoadmap();
            roadmaps.push_back(rm);
        }
        else
        {
            std::cout << "No Solution Found" << std::endl;
        }
    }
    // An attempt to cut down on the neighbors we are searching through in dRRT
    //prmPtr->setMaxNearestNeighbors(10);

    //solve the problem:

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
        // cast to compound state space
        ob::CompoundStateSpace * compStateSpace = stateSpace->as<ob::CompoundStateSpace>();     

        // allocate space for compound state
        ob::CompoundState *compState = compStateSpace->allocState()->as<ob::CompoundState>();   

        for (size_t j = 0; j < compositeState[i].size(); ++j) {

            // extract state from matrix of states
            ob::ScopedState<> tempState = compositeState[i][j];

            // copy over the state as a substate of a composite state                                
            compStateSpace->getSubspace(j)->copyState(compState->components[j], tempState.get());  
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
    compound->setPlanner(std::make_shared<ompl::geometric::RRT>(si));

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


int main(int, char **)
{
    int choice;
    do
    {
        std::cout << "LR or Clock? " << std::endl;
        std::cout << " (1) LR" << std::endl;
        std::cout << " (2) Clock" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    if (choice == 1)
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
        compositeSolve(r1, r2, r3, r4, r1SX, r1SY, r1GX, r1GY, r2SX, r2SY, r2GX, r2GY, r3SX, r3SY, r3GX, r3GY, r4SX, r4SY, r4GX, r4GY, obstacles);

        // Now, we want to plan with our implicit search dRRT, using our individual PRM roadmaps (not doing tensor product)

        // Make a compound state space with the spaces of our 4 robot PRMS:
        auto stateSpace = r1->getStateSpace() + r2->getStateSpace() + r3->getStateSpace() + r4->getStateSpace();
        
        // Initialize a simple setup pointer:
        og::SimpleSetupPtr compound = std::make_shared<og::SimpleSetup>(stateSpace);

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

        // Retrieving space information pointer from simple setup
        ob::SpaceInformationPtr si = compound->getSpaceInformation();

        // TODO: uncomment or comment it out to show the different behaviors with obstacles and without obstacles
        compound->setStateValidityChecker(
            [si, &obstacles](const ob::State* state) { return isStateValid(si, state, obstacles); }
        );

        // Set our planner as dRRT
        auto planner = std::make_shared<og::dRRT>(si);
        
        // Trying to set the members holding vector of states for each robot by accessing our planner (dRRT) and calling setRobotNodes
        std::vector<og::PRM::Graph> roadmaps = {r1RM, r2RM, r3RM, r4RM};
        std::vector<og::SimpleSetupPtr> ssPtrs = {r1, r2, r3, r4};
        planner->setRobotPRMs(roadmaps);
        planner->setRobotNodes();
        planner->setIndivSpaceInfo(ssPtrs);
        // planner->setRobotNodes(r1RM_nodes, r2RM_nodes, r3RM_nodes, r4RM_nodes);

        // ADDED planner and set robot nodes --> works now
        compound->setPlanner(planner);

        compound->setup();


        // Solve with dRRT (obviously not working yet)
        ob::PlannerStatus solved = compound->solve(20);

        std::ofstream solution("solution_path.txt");

        if (solved)
        {
            std::cout << "Found Solution:" << std::endl;

            auto path = compound->getSolutionPath();
            path.printAsMatrix(std::cout);
            
            path.printAsMatrix(solution);
        }
        else{
            std::cout << "No Solution Found" << std::endl;
        }


    }
    else if (choice == 2)
    {
        // Open space environment, no obstacles
        // TODO: will this create a segmentation fault?
        std::vector<Rectangle> obstacles;
        
        // Creating the varius robot start and goal states: 
        // Now doing the clock environment with robots standing in a circle of radius 4  

        double r1SX = 5.0;
        double r1SY = 9.0;
        double r1GX = 5.0;
        double r1GY = 1.0;

        double r2SX = 7.83;
        double r2SY = 7.83;
        double r2GX = 2.17;
        double r2GY = 2.17;

        double r3SX = 9.0;
        double r3SY = 5.0;
        double r3GX = 1.0;
        double r3GY = 5.0;

        double r4SX = 7.83;
        double r4SY = 2.17;
        double r4GX = 2.17;
        double r4GY = 7.83;

        double r5SX = 5.0;
        double r5SY = 1.0;
        double r5GX = 5.0;
        double r5GY = 9.0;

        double r6SX = 2.17;
        double r6SY = 2.17;
        double r6GX = 7.83;
        double r6GY = 7.83;

        double r7SX = 1.0;
        double r7SY = 5.0;
        double r7GX = 9.0;
        double r7GY = 5.0;

        double r8SX = 2.17;
        double r8SY = 7.83;
        double r8GX = 7.83;
        double r8GY = 2.17;

        og::SimpleSetupPtr r1 = createClockRobot(r1GX, r1GY, r1SX, r1SY, obstacles);
        og::SimpleSetupPtr r2 = createClockRobot(r2GX, r2GY, r2SX, r2SY, obstacles);
        og::SimpleSetupPtr r3 = createClockRobot(r3GX, r3GY, r3SX, r3SY, obstacles);
        og::SimpleSetupPtr r4 = createClockRobot(r4GX, r4GY, r4SX, r4SY, obstacles);

        og::SimpleSetupPtr r5 = createClockRobot(r5GX, r5GY, r5SX, r5SY, obstacles);
        og::SimpleSetupPtr r6 = createClockRobot(r6GX, r6GY, r6SX, r6SY, obstacles);
        og::SimpleSetupPtr r7 = createClockRobot(r7GX, r7GY, r7SX, r7SY, obstacles);
        og::SimpleSetupPtr r8 = createClockRobot(r8GX, r8GY, r8SX, r8SY, obstacles);

        std::vector<og::SimpleSetupPtr> ssPtrsClock = {r1, r2, r3, r4, r5, r6, r7, r8};
        std::vector<og::PRM::Graph> roadmaps;
        planClockRobots(ssPtrsClock, roadmaps);

        // Now, we want to plan with our implicit search dRRT, using our individual PRM roadmaps (not doing tensor product)

        // Make a compound state space with the spaces of our 4 robot PRMS:
        auto stateSpace = r1->getStateSpace() + r2->getStateSpace() + r3->getStateSpace() + r4->getStateSpace() + r5->getStateSpace() + r6->getStateSpace() + r7->getStateSpace() + r8->getStateSpace();
        
        // Initialize a simple setup pointer:
        og::SimpleSetupPtr compound = std::make_shared<og::SimpleSetup>(stateSpace);

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

        start->as<ob::RealVectorStateSpace::StateType>(4)->values[0] = r5SX;
        start->as<ob::RealVectorStateSpace::StateType>(4)->values[1] = r5SY;

        start->as<ob::RealVectorStateSpace::StateType>(5)->values[0] = r6SX;
        start->as<ob::RealVectorStateSpace::StateType>(5)->values[1] = r6SY;

        start->as<ob::RealVectorStateSpace::StateType>(6)->values[0] = r7SX;
        start->as<ob::RealVectorStateSpace::StateType>(6)->values[1] = r7SY;

        start->as<ob::RealVectorStateSpace::StateType>(7)->values[0] = r8SX;
        start->as<ob::RealVectorStateSpace::StateType>(7)->values[1] = r8SY;

        ob::ScopedState<ob::CompoundStateSpace> goal(compound->getSpaceInformation());
        goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = r1GX;
        goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = r1GY;

        goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = r2GX;
        goal->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = r2GY;

        goal->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = r3GX;
        goal->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = r3GY;

        goal->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = r4GX;
        goal->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = r4GY;

        goal->as<ob::RealVectorStateSpace::StateType>(4)->values[0] = r5GX;
        goal->as<ob::RealVectorStateSpace::StateType>(4)->values[1] = r5GY;

        goal->as<ob::RealVectorStateSpace::StateType>(5)->values[0] = r6GX;
        goal->as<ob::RealVectorStateSpace::StateType>(5)->values[1] = r6GY;

        goal->as<ob::RealVectorStateSpace::StateType>(6)->values[0] = r7GX;
        goal->as<ob::RealVectorStateSpace::StateType>(6)->values[1] = r7GY;

        goal->as<ob::RealVectorStateSpace::StateType>(7)->values[0] = r8GX;
        goal->as<ob::RealVectorStateSpace::StateType>(7)->values[1] = r8GY;

        compound->setStartAndGoalStates(start, goal);

        // Retrieving space information pointer from simple setup
        ob::SpaceInformationPtr si = compound->getSpaceInformation();

        // TODO: uncomment or comment it out to show the different behaviors with obstacles and without obstacles
        compound->setStateValidityChecker(
            [si, &obstacles](const ob::State* state) { return isStateValid(si, state, obstacles); }
        );

        // Set our planner as dRRT
        auto planner = std::make_shared<og::dRRT>(si);
        
        // Trying to set the members holding vector of states for each robot by accessing our planner (dRRT) and calling setRobotNodes
        planner->setRobotPRMs(roadmaps);
        planner->setRobotNodes();
        planner->setIndivSpaceInfo(ssPtrsClock);

        // ADDED planner and set robot nodes --> works now
        compound->setPlanner(planner);

        compound->setup();


        // Solve with dRRT (obviously not working yet)
        ob::PlannerStatus solved = compound->solve(20);

        std::ofstream solution("solution_path.txt");

        if (solved)
        {
            std::cout << "Found Solution:" << std::endl;

            auto path = compound->getSolutionPath();
            path.printAsMatrix(std::cout);
            
            path.printAsMatrix(solution);
        }
        else{
            std::cout << "No Solution Found" << std::endl;
        }

    }
    
}