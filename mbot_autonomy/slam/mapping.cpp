#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
    : kMaxLaserDistance_(maxLaserDistance), kHitOdds_(hitOdds), kMissOdds_(missOdds), initialized_(false)
{
}

void Mapping::updateMap(const mbot_lcm_msgs::lidar_t &scan, const mbot_lcm_msgs::pose2D_t &pose, OccupancyGrid &map) {
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;
    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan
    // Hint: Consider both the cells the laser hit and the cells it passed through.

    for (auto &ray : movingScan) {
        scoreEndpoint(ray, map);
    } 
    
    for (auto &ray : movingScan) {
        scoreRay(ray, map);
    } 

    previousPose_ = pose;
}

/*each cell holds the probability of its occupancy, where probability is presented in log odds. Based on ray data 
from LiDAR, we calculate the likelihood of occupancy of the cell at each scan and then increase or 
decrease the odds of the cell being occupied.
*/
/*An occupancy grid maps the space surrounding the robot by assigning probabilities to each cell, 
indicating whether they are occupied or free
*/
//Implement the algorithm to score the cell that the laser endpoint hits
void Mapping::scoreEndpoint (const adjusted_ray_t &ray, OccupancyGrid &map) {
    if (ray.range < kMaxLaserDistance_) {
        Point<float> rayStart = global_position_to_grid_position (ray.origin, map);

        //find end cell of the ray
        Point<int> rayEnd;
        rayEnd.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayEnd.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        //if end cell is in map grip then update the cell odds (is obstacle)
        if (map.isCellInGrid(rayEnd.x, rayEnd.y)) {
            increaseCellOdds(rayEnd.x, rayEnd.y, map);
        }
    }
}

//Implement the algorithm to score the cells that the laser ray passes through
void Mapping::scoreRay(const adjusted_ray_t &ray, OccupancyGrid &map) {
    if (ray.range < kMaxLaserDistance_) {
        std::vector<Point<int>> freeCells = bresenham(ray, map);

        for (size_t i = 0; i < freeCells.size() - 1; i++) {
            if (map.isCellInGrid(freeCells[i].x, freeCells[i].y)) {
                decreaseCellsOdds(freeCells[i].x, freeCells[i].y));
            }
        }
    }
}


void Mapping::increaseCellOdds (int x, int y, OccupancyGrid &map) {
    //increase the odds of cell at (x, y)
    if (!initialized) {
    } else if (127 - map(x, y) > kHitOdds_) {
        map (x, y) += kHitOdds_;
    } else {
        map(x, y) = 127;
    }

void Mapping::decreaseCellsOdds (int x, int y, OccupancyGrid &map) {
    //decrease the odds of cell at (x, y)
    if (!initialized_) {
    } else if (map(x, y) + 128 > kMissOdds_) {
        map(x, y) -= kMissOdds_;
    } else {
        map(x, y) = -128;
    }
}

//Implement the Bresenham's line algorithm to 
//find cells touched by the ray and returns a vector of map celss to check.
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t &ray, const OccupancyGrid &map) {
    //compute start and end points
    Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
    int x0 = static_cast<int>(rayStart.x);
    int y0 = static_cast<int>(rayStart.y);
    int x1 = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    int y1 = static_cast<int>((ray.range * std::sin(ray.theta) * map.celssPerMeter()) + rayStart.y);

    //compute the absolute difference and signs for start and end points
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    //initialize the error term
    int err = dx - dy;

    //create a vector to sotre the free cells.
    std::vector<Point<int>> freeCells;

    //current point
    Point<int> startPoint;
    startPoint.x = x0;
    startPoint.y = y0;
    int x = x0;
    int y = y0;
    //Add initial point
    freeCells.push_back(startPoint);

    //Main Bresenham's Line Algorithm loop
    while (x != x1 || y != y1) {
        int e2 = 2 * err;
        if (e2 >= -dy) {    //Move in x direction
            err -= dy;
            x += sx;
        }
        if (e2 <= dx) {    //Move in y direction
            err += dx;
            y += sy;
        }
        Point<int> freePoint;
        freePoint.x = x;
        freePoint.y = y;
        freeCells.push_back(freePoint);
    }
    return freeCells;
}
        

//an alternative approach to find cells touched by the ray.
std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t &ray, const OccupancyGrid &map) {
    float stepSize = 0.5 * map.metersPerCell();      //0.05 meters for each cell default
    float maxRange = ray.range;
    int numSteps = static_cast<int>(maxRange / stepSize);

    std::vector<Point<int>> touchedCells;
    Point<int> prevCell;
    Point<double> rayStart = global_position_to_grid_position (ray.origin, map);

    for (int i = 0; i < numSteps; i++) {
        Point<int> rayCell;
        rayCell.x = static_cast<int> (i * stepSize * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int> (i * stepSize * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);
        if (map.isCellInGrid(rayCell.x, rayCell.y)) {
            if (rayCell != prevCell) { //avoid duplicates
                touchedCells.push_back(rayCell);
            }
            prevCell = rayCell;
        }
    }
    return touchedCells; //placeholder
}
