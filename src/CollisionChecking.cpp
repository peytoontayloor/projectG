///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Author: Ryan Luna
//////////////////////////////////////

# include <vector>
# include <cstddef>
# include <cmath>

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

bool isValidStatePoint(const ompl::base::State *state, const std::vector<Rectangle> &obstacles) {
    const ompl::base::RealVectorStateSpace::StateType *R2State
        = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = R2State->values[0];
    double y = R2State->values[1];

    for (size_t i = 0; i < obstacles.size(); ++i) {
        if (rectangleToAABB(obstacles[i]).pointInsideAABB(x, y)) {
            return false;
        }
    }

    return true;
}