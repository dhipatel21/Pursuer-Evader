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


float inverse_sensor_model(float x, float y, float theta, float x1, float y1, float range) {
    Point<int> pointA = {x, y};
    Point<int> pointB = {x1, y1};

    float r = distance_between_points(pointA, pointB);
    float phi = atan2(pointB.y - pointA.y, pointB.x - pointA.x) - theta;
    float alpha = 0.1; // dimension of cell is 10x10 cm
    float z_max = 10; // max allowable distance is 10 m (???? Might need to confirm this)

    // if the distance to the cell is greater than the maximum allowable range or the measured range
    // then it is beyond an obstacle and/or unknown
    if (r > std::min(z_max, range + alpha/2)) { 
        return 0;
    }
    // if the measured range to obstacle is less than the maximum allowable range AND the difference
    // between the measured range and the distance to that cell is ~0, then the cell is an obstacle
    else if ((range < z_max) && (abs(r - range) < alpha/2)) { 
        return 1;
    }
    // if distance to cell is less than measured range to obstacle, then it is unoccupied
    else if (r <= range) {
        return -1;
    }
    // return unkown if we don't hit any of the other cases 
    else {
        return 0;
    }
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here //////////////////////

    // Pseudo code From Lecture 07 slide 22 --> need to include movingScan
    // MovingLaserScan movingScan(scan, previousPose, pose);

    // for (auto& ray : movingScan) {
    //     scoreEndpoint(ray, map);
    // }

    // for (auto& ray : movingScan) {
    //     scoreRay(ray, map);
    // }
    // previousPose_ = pose;

    // From Lecture 07 slide 11 --> needs to be fixed, include inverse sensor model?

    float x = pose.x;
    float y = pose.y;
    float theta = pose.theta;

    for (int i = 0; i < scan.num_ranges; ++i) {
        float range = scan.ranges[i];
        Point<float> endpoint;

        endpoint.x = x + range * cos(pose.theta + scan.thetas[i]);
        endpoint.y = y + range * sin(pose.theta + scan.thetas[i]);
        int x1 = endpoint.x / map.metersPerCell();
        int y1 = endpoint.y / map.metersPerCell();

        // Bresenham's line algorithm
        int dx = abs(x1 - x);
        int dy = abs(y1 - y);
        int sx = x < x1 ? 1 : -1;
        int sy = y < y1 ? 1 : -1;
        int err = dx - dy;

        while (x1 != x || y1 != y) {
            // Update odds
            
            int oldValue = map.logOdds(x, y);
            int newValue = oldValue;

            if (map.isCellInGrid(x, y)) {
                newValue += inverse_sensor_model(x, y, theta, x1, y1, range) - log(map.logOdds(x, y) / (1 - map.logOdds(x, y)));
                // keep logOdds in the range [-127, 127]
                if (newValue > 127) {
                    newValue = 127;
                }
                else if (newValue < -127) {
                    newValue = -127;
                }

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
