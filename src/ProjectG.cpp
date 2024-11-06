
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

# include "CollisionChecking.h"
# include "dRRT.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

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

std::vector<std::vector<ob::ScopedState<>>> createCompositeRM(ob::StateSpacePtr r1Space, ob::StateSpacePtr r2Space, ob::StateSpacePtr r3Space, ob::StateSpacePtr r4Space)
{
    // Construct composite roadmap by going through each robots roadmap vertices, and creating a configuration of a higher dimension
    // So, if we have 4 robots with the vertices, r1, r2, r3, r4, from each robots PRM, the composite space would be (r1, r2, r3, r4) for this point

    // TODO: investigate how to handle the fact that some roadmaps have more states than others? --> I still am confused about this, should ask professor Kavraki in our meeting :) 

    // Only add to composite roadmap if each position for the states are collision free amongst the robots
    
    // Vector of Vectors, we want the main 'list' to be composite states, and the composite states are a vector holding the configuration for each robot
    std::vector<std::vector<ob::ScopedState<>>> compStates;

    // First, loop through each of the PRM's (still not sure what to do when we have no more states in one of the robots list of configs)
    // TODO: right now, stopping when smallest vector stops, feel like this is iffy, need to investigate.


    // Initialize indices to 0
    size_t i1 = 0;
    size_t i2 = 0;
    size_t i3 = 0;
    size_t i4 = 0;

    while((i1 < r1RM_nodes.size()) and (i2 < r2RM_nodes.size()) and (i3 < r3RM_nodes.size()) and (i4 < r4RM_nodes.size()))
    {
        // Now, check for collisions for the robot's positions, if no collisions, add to compStates, else continue looping
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
        i1++;
        i2++;
        i3++;
        i4++;
    }

    return compStates;
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

    //og::PRM::Graph composite = createCompositeRM();

    // Having composite RM return a vector of vectors of states so that we can sample from those in dRRT
    std::vector<std::vector<ob::ScopedState<>>> compositeState = createCompositeRM(r1->getStateSpace(), r2->getStateSpace(), r3->getStateSpace(), r4->getStateSpace());

    // This print is for checking that the states in our composite 'roadmap' are equal to the ammount of states in our smallest PRM roadmap
    // It will be a little less than that if there are collisions :)
    
    // std::cout << compositeState.size() << std::endl;

    ob::CompoundStateSpace compoundStateSpace;
    compoundStateSpace.addSubspace(r1->getStateSpace(), 1);
    compoundStateSpace.addSubspace(r2->getStateSpace(), 1);
    compoundStateSpace.addSubspace(r3->getStateSpace(), 1);
    compoundStateSpace.addSubspace(r4->getStateSpace(), 1);

    r1->setPlanner(std::make_shared<ompl::geometric::RRT>(r1->getSpaceInformation()));
    // r2->setPlanner(std::make_shared<ompl::geometric::RRT>(r1->getSpaceInformation()));
    // r3->setPlanner(std::make_shared<ompl::geometric::RRT>(r1->getSpaceInformation()));
    // r4->setPlanner(std::make_shared<ompl::geometric::RRT>(r1->getSpaceInformation()));

    ob::PlannerStatus solved = r1->solve(20);
    std::ofstream solution("path_RRT.txt");

    if (solved)
    {
        std::cout << "Found Solution:" << std::endl;

        auto path = r1->getSolutionPath();
        path.printAsMatrix(solution);
    }
    else{
        std::cout << "No Solution Found" << std::endl;
    }

}