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


float inverse_sensor_model(const pose_xyt_t& pose) {
    Point<int> pointA = {pose.x, pose.y};
    // xi and yi are the center of mass of mi
    Point<int> pointB = {xi, yi};
    float r = distance_between_points(pointA, pointB);
    float angle_to_cell = atan2(pointB.y - pointA.y, pointB.x - pointA.x) - pose.theta;
    float k = 
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    int x0 = pose.x;
    int y0 = post.y;

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
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = poseX < endX ? 1 : -1;
        int sy = poseY < endY ? 1 : -1;
        int err = dx - dy;

        while (x1 != x0 || y1 != y0) {
            // Update odds
            int oldValue = map.logOdds(x0, y0);
            int newValue = oldValue;
            if (map.isCellInGrid(x0, y0)) {
                newValue += inverse_sensor_model() - l0;
                map.setLogOdds(x0, y0, newValue);
            }

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
}
