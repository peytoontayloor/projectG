# include <utility>
# include <vector>
# include "Robot.h"

// For new state point validity checker
#include <ompl/base/spaces/SE2StateSpace.h>

// Axis aligned bounding box
struct AABB
{
    double minX, minY;
    double maxX, maxY;

    bool pointInsideAABB(double x, double y, double rad) const;
    bool pointInsideAABB(double x, double y) const;
};

struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};

#define MAKE_POINT(x,y) std::make_pair(x, y)
typedef std::pair<double, double> Point2D;

// Transforms from rectangles to axis-aligned bounding box representation
AABB rectangleToAABB(const Rectangle &obstacle);

// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles);

bool isValidStatePoint(const ompl::base::State* state, const std::vector<Rectangle>& obstacles);

std::vector<int> robotRobotCollisionCheck(Robot r1, std::vector<Robot>& robots);
