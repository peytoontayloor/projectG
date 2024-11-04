
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
namespace oc = ompl::control;

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

// Creating robots for the square environment where robots on left and right sides swap
void createRobotsLR(std::vector<Rectangle> & obstacles){
    // robot 1 (from diagram labeling, top left corner going to to top right corner)
    std::cout << "starting" << std::endl;
    auto topLeftRobot = std::make_shared<Robot>();

    // robot 2 (from diagram labeling, top right corner going to to top left corner)
    auto topRightRobot = std::make_shared<Robot>();

    // robot 3 (from diagram labeling, lower right corner going to to lower left corner)
    auto lowerLeftRobot= std::make_shared<Robot>();

    // robot 4 (from diagram labeling, lower right corner going to to lower left corner)
    auto lowerRightRobot = std::make_shared<Robot>();

    topLeftRobot->setPRMPlanner(2.0, 8.0, 6.0, 8.0);
    topRightRobot->setPRMPlanner(6.0, 8.0, 2.0, 8.0);
    lowerLeftRobot->setPRMPlanner(2.0, 2.0, 6.0, 2.0);
    lowerRightRobot->setPRMPlanner(6.0, 6.0, 2.0, 2.0);

    std::cout << "createLowerLeftRobot" << std::endl;

}

// Creating robots for the clock swapping 
void createRobotsClock(){
    // TO DO: creating robots for the clock swapping
    // auto robot1 = std::make_shared<Robot>();
    // robot1->setPRMPlanner

}


int main(int, char **)
{
    std::vector<Rectangle> obstacles;
    createLREnvironment(obstacles);
    createRobotsLR(obstacles);

}