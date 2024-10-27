
# include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/base/SpaceInformation.h"
# include <ompl/base/spaces/RealVectorStateSpace.h>
# include <ompl/geometric/SimpleSetup.h>
# include <ompl/base/ScopedState.h>
# include <ompl/base/spaces/SE3StateSpace.h>
# include <ompl/base/ProblemDefinition.h>

# include "CollisionChecking.h"

# include "dRRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

 bool isStateValid(const ob::State *state)
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
    return true;
 }
struct Robot{

    void setPRMPlanner(double startX, double startY, double goalX, double goalY){
        
        // Set xy bounds for robot based on diagram in handout
        auto xy_space(std::make_shared<ob::RealVectorStateSpace>(2));
        ob::RealVectorBounds xy_bounds(2);
        xy_bounds.setLow(0, 1);
        xy_bounds.setHigh(0, 7);
        xy_bounds.setLow(1, 1);
        xy_bounds.setHigh(1, 9);
        xy_space->setBounds(xy_bounds);

        auto si = std::make_shared<ompl::base::SpaceInformation>(xy_space);

        // Set start and goal states for robot based on input 
        ob::ScopedState<ob::RealVectorStateSpace> start(si);
        ob::ScopedState<ob::RealVectorStateSpace> goal(si);

        start[0] = startX;
        start[1] = startY;

        goal[0] = goalX;
        goal[1] = goalY;

        si->setStateValidityChecker(isStateValid);
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);

        auto planner = std::make_shared<ompl::geometric::PRM>(si);
        planner->setup();
        si->printSettings(std::cout);
        
        ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);

    }
};

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
    auto topLeftRobot = std::make_shared<Robot>();
    topLeftRobot->setPRMPlanner(2, 8, 6, 8);

    // robot 2 (from diagram labeling, top right corner going to to top left corner)
    auto topRightRobot = std::make_shared<Robot>();
    topLeftRobot->setPRMPlanner(6, 8, 2, 8);

    // robot 3 (from diagram labeling, lower right corner going to to lower left corner)
    auto lowerLeftRobot = std::make_shared<Robot>();
    topLeftRobot->setPRMPlanner(2, 2, 6, 2);

    // robot 4 (from diagram labeling, lower right corner going to to lower left corner)
    auto lowerRightRobot = std::make_shared<Robot>();
    topLeftRobot->setPRMPlanner(6, 2, 2, 2);

}

// Creating robots for the clock swapping 
void createRobotsClock(){
    // TO DO: creating robots for the clock swapping


}