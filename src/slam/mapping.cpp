#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


float inverse_sensor_model(int x, int y, int x1, int y1, float range) {
    // !!!!!!!!!!!!!!!!!
    // RETURN VALUES ARE TBD - what exactly is l0, l occ, and l empt?
    // how to incorporate log odds in a way that makes sense
    // !!!!!!!!!!!!!!!!

    Point<int> pointA = {x, y};
    Point<int> pointB = {x1, y1};

    float dist_to_cell = distance_between_points(pointA, pointB);
    float angle_to_cell = atan2(pointB.y - pointA.y, pointB.x - pointA.x) - pose.theta;

    float alpha = 0.1; // dimension of cell is 10x10 cm
    float z_max = 10; // max allowable distance is 10 m (???? Might need to confirm this)

    // if the distance to the cell is greater than the maximum allowable range or the measured range
    // then it is beyond an obstacle and/or unknown
    if dist_to_cell > min(z_max, range + alpha/2) { 
        return 0.5;
    }

    // if the measured range to obstacle is less than the maximum allowable range AND the difference
    // between the measured range and the distance to that cell is ~0, then the cell is an obstacle
    if (range < z_max) && (abs(dist_to_cell - range) < alpha/2) { 
        return 1;
    }

    // if distance to cell is less than measured range to obstacle, then it is unoccupied
    if (dist_to_cell < range) {
        return 0;
    }
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    int x = pose.x;
    int y = pose.y;

    // From Lecture 07 slide 22
    // MovingLaserScan movingScan(scan, previousPose, pose);

    // From Lecture 07 slide 11 --> needs to be fixed, include inverse sensor model?
    for (int i = 0; i < scan.num_ranges; ++i)
    {
        float range = scan.ranges[i];

        float endpointX = x0 + range * cos(pose.theta + scan.thetas[i]);
        float endpointY = y0 + range * sin(pose.theta + scan.thetas[i]);
        int x1 = endpointX / map.metersPerCell();
        int y1 = endpointY / map.metersPerCell();

        // Bresenham's line algorithm
        int dx = abs(x1 - x);
        int dy = abs(y1 - y);
        int sx = poseX < endX ? 1 : -1;
        int sy = poseY < endY ? 1 : -1;
        int err = dx - dy;

        while (x1 != x || y1 != y) {
            // Update odds
            
            int oldValue = map.logOdds(x, y);
            int newValue = oldValue;

            if (map.isCellInGrid(x, y)) {
                newValue += inverse_sensor_model(x, y, x1, y1, range);
                map.setLogOdds(x, y, newValue);
            }

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
}
