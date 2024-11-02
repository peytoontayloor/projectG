
# include <ompl/geometric/planners/prm/PRM.h>
# include <ompl/base/SpaceInformation.h>
# include <ompl/base/spaces/RealVectorStateSpace.h>
# include <ompl/geometric/SimpleSetup.h>
# include <ompl/base/ScopedState.h>
# include <ompl/base/spaces/SE3StateSpace.h>
# include <ompl/base/ProblemDefinition.h>
# include <fstream>
# include <iostream>
# include "CollisionChecking.h"
# include <ompl/base/PlannerStatus.h>

# include "dRRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

void createSquareEnvironment(){
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
}

// Creating robots for the square environment where robots on left and right sides swap
void createRobotsLR(){
    // robot 1 (from diagram labeling, top left corner going to to top right corner)
    std::cout << "starting" << std::endl;
    Robot topLeftRobot(2, 8, 0.3, 1);

    // robot 2 (from diagram labeling, top right corner going to to top left corner)
    Robot topRightRobot(6, 8, 0.3, 2);

    // robot 3 (from diagram labeling, lower right corner going to to lower left corner)
    Robot lowerLeftRobot(2, 2, 0.3, 3);

    // robot 4 (from diagram labeling, lower right corner going to to lower left corner)
    Robot lowerRightRobot(6, 2, 0.3, 3);

    std::vector<Robot> robots;
    robots.push_back(topLeftRobot);
    robots.push_back(topRightRobot);
    robots.push_back(lowerLeftRobot);
    robots.push_back(lowerRightRobot);

    topLeftRobot.setPRMPlanner(6, 8);
    topRightRobot.setPRMPlanner(2, 8);
    lowerLeftRobot.setPRMPlanner(6, 2);
    lowerRightRobot.setPRMPlanner(2, 2);

    std::cout << "createLowerLeftdRobot" << std::endl;

}

// Creating robots for the clock swapping 
void createRobotsClock(){
    // TO DO: creating robots for the clock swapping
    // auto robot1 = std::make_shared<Robot>();
    // robot1->setPRMPlanner

}

void planLR(){
    createRobotsLR();

}

void planClock(){

}

int main(int, char **)
{
    int environment;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) LR Robots" << std::endl;
        std::cout << " (2) Clock Robots" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            planLR();
            break;
        case 2:
            planClock();
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    return 0;
}