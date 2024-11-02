# include <vector>

class Robot
{
    public: 
        int id;
        Robot(double x, double y, double radius, int id);
        ~Robot();
        void setPRMPlanner(double goalX, double goalY);
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