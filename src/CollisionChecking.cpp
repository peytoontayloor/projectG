///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Author: Ryan Luna
//////////////////////////////////////

# include <vector>
# include <cstddef>

#include "CollisionChecking.h"

// The following methods were copied from CollisionChecking.cpp in Project 4. No other code has been changed
// Axis aligned bounding box

// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    for(size_t i = 0; i < obstacles.size(); ++i)
    {
        if (x >= obstacles[i].x && x <= obstacles[i].x + obstacles[i].width)
            if (y >= obstacles[i].y && y <= obstacles[i].y + obstacles[i].height)
                return false;
    }
    return true;
}

// The following methods were copied from CollisionChecking.cpp in Project 3. No other code has been changed
// Axis aligned bounding box
bool AABB::pointInsideAABB(double x, double y) const {
    return x >= minX && x <= maxX && y >= minY && y <= maxY;
}

AABB rectangleToAABB(const Rectangle &obstacle) {
    AABB rect { obstacle.x, obstacle.y, obstacle.x + obstacle.width, obstacle.y + obstacle.height };
    return rect;
}

/*
Added checking robot for collision with every other robot
Given a robot, returns a list of ids of robots it is in collision with 
*/ 
std::vector<int> robotRobotCollisionCheck(Robot r1, std::vector<Robot>& robots)
{
    std::vector<int> collisionIds = {};
    for (int i = 0; i < robots.size(); i ++)
    {
        Robot r_i = robots[i];
        if (r1.id != r_i.id){
            std::vector<Rectangle> obstacles;
            Rectangle robot_i;
            robot_i.x = r_i.getX();
            robot_i.y = r_i.getY();
            robot_i.width = r_i.getRadius();
            robot_i.height = r_i.getRadius();
            obstacles.push_back(robot_i);
            bool valid = isValidPoint(r_i.getX(), r_i.getY(), obstacles);
            if (!valid){
                collisionIds.push_back(r_i.id);
            }
        }
    }
    return collisionIds;
}