
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

# include "CollisionChecking.h"
# include "dRRT.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

og::PRM::Graph r1RM;
og::PRM::Graph r2RM;
og::PRM::Graph r3RM;
og::PRM::Graph r4RM;

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
            prmPtr->getRoadmap();
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

    // TODO: now that we have these paths, going to try and get the full roadmaps for each of the robots instead of just solution paths

}