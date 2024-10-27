
# include <ompl/geometric/planners/prm/PRMstar.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class Robot(){

    Robot();
    
    ~Robot();

    ompl::geometric::PRMstar planner;

    void setPRMStarPlanner(double startX, double startY, double goalX, double goalY){

        // Set xy bounds for robot based on diagram in handout
        auto xy_space(std::make_shared<ob::RealVectorStateSpace>(2));
        ob::RealVectorBounds xy_bounds(2);
        xy_bounds.setLow(0, 1);
        xy_bounds.setHigh(0, 7);
        xy_bounds.setLow(1, 1);
        xy_bounds.setHigh(1, 9);
        xy_space->setBounds(xy_bounds);

        auto ss = std::make_shared<ob::SimpleSetup>();

        oc::SpaceInformationPtr si = ss ->getSpaceInformation();
        planner = ompl::geometric::PRMstar(si);

        si->setPropagationStepSize(0.3);

        // Set start and goal states for robot based on input 
        ob::ScopedState<ob::RealVectorStateSpace> start(si);
        ob::ScopedState<ob::RealVectorStateSpace> goal(si);

        start[0] = startX;
        start[1] = startY;

        goal[0] = goalX;
        goal[1] = goalY;

        ss->setStartAndGoalStates(start, goal, 0.05);
        ss->setup();
        ss->print();
        ob::PlannerStatus solved = ss->solve(20.0);

    }
}

void createEnvironment(){
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

void createRobots(){
    // robot 1 (from diagram labeling, top left corner going to to top right corner)
    auto topLeftRobot = std::make_shared<Robot>();
    topLeftRobot->setPRMStarPlanner(2, 8, 6, 8);

    // robot 2 (from diagram labeling, top right corner going to to top left corner)
    auto topRightRobot = std::make_shared<Robot>();
    topLeftRobot->setPRMStarPlanner(6, 8, 2, 8);

    // robot 3 (from diagram labeling, lower right corner going to to lower left corner)
    auto lowerRightRobot = std::make_shared<Robot>();
    topLeftRobot->setPRMStarPlanner(2, 2, 6, 2);

    // robot 4 (from diagram labeling, lower right corner going to to lower left corner)
    auto lowerRightRobot = std::make_shared<Robot>();
    topLeftRobot->setPRMStarPlanner(6, 2, 2, 2);

}