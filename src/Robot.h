# include <vector>

class Robot
{
    public: 
        int id;
        Robot();
        ~Robot();
        void setPRMPlanner(double startX, double startY, double goalX, double goalY);
        double getX();
        double getY();
        double getRadius();
        std::vector<Robot> getEnvRobots();

        void setX(double);
        void setY(double);
        void setRadius(double);
        void setEnvRobots(std::vector<Robot> robots);

    private: 
        double x;
        double y;
        double radius;
        std::vector<Robot> robots;

};