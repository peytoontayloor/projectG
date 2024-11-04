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

namespace ob = ompl::base;
namespace oc = ompl::control;

Robot::Robot(){}

// Robot::Robot(double x_param, double y_param, double radius_param, int id_param){
//     id = id_param;
//     x = x_param;
//     y = y_param;
//     radius = radius_param;
// }

Robot::~Robot(){
    
}

// double Robot::getX(){
//     return x;
// }

// double Robot::getY(){
//     return y;
// }

// double Robot::getRadius(){
//     return radius;
// }

// void Robot::setX(double x_param){
//     x = x_param;
// }

// void Robot::setY(double y_param){
//     y = y_param;
// }

// void Robot::setRadius(double radius_param){
//     radius = radius_param;
// }

bool isStateValid(const ob::SpaceInformationPtr si, double x, double y, double radius, std::vector<Rectangle>& obstacles)
{
     // cast the abstract state type to the type we expect
    //  const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

     // extract the first component of the state and cast it to what we expect
    //  const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
  
     // extract the second component of the state and cast it to what we expect
    //  const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
  
     // check validity of state defined by pos & rot
  
     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    //  return (const void*)rot != (const void*)pos;
    // return si->satisfiesBounds(state) && robotRobotCollisionCheck(x, y, radius, robots);
    // return si->satisfiesBounds(state) && isValidPoint(x, y, obstacles);
    return true;
}

void Robot::setPRMPlanner(double startX, double startY, double goalX, double goalY){

    std::ofstream solution ("path_" + std::to_string(id) + ".txt");

    // Set xy bounds for robot based on diagram in handout
    auto xy_space(std::make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds xy_bounds(2);
    xy_bounds.setLow(0, 1);
    xy_bounds.setHigh(0, 7);
    xy_bounds.setLow(1, 1);
    xy_bounds.setHigh(1, 9);
    xy_space->setBounds(xy_bounds);

    auto ss = std::make_shared<ompl::geometric::SimpleSetup>(xy_space);
    auto si = ss->getSpaceInformation();

    // Set start and goal states for robot based on input 
    ob::ScopedState<> start(si);
    ob::ScopedState<> goal(si);

    start[0] = startX;
    start[1] = startY;

    goal[0] = goalX;
    goal[1] = goalY;

    // ss->setStateValidityChecker(std::bind(isStateValid(si, x, y, radius, obstacles)));
    ss->setStartAndGoalStates(start, goal);
    ss->setup();
    ss->print();

    auto planner = std::make_shared<ompl::geometric::PRM>(si);
    ss->setPlanner(planner);
    
    ompl::base::PlannerStatus solved = ss->solve(5.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss->getSolutionPath().printAsMatrix(solution);
        std::cout << "testing print" << std::endl;
        //ob::PlannerData plannerData(ss->getSpaceInformation());
        //ss->getPlannerData(plannerData);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}
