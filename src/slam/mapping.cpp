#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance), kHitOdds_(hitOdds), kMissOdds_(missOdds)
{
    pose_xyt_t previousPose_ = pose_xyt_t();
    previousPose_.theta = 0;
    previousPose_.x = 0;
    previousPose_.y = 0;
    previousPose_.utime = 0;
}


CellOdds inverse_sensor_model(float x, float y, float theta, float x1, float y1, float range) {
    Point<float> pointA = {x, y};
    Point<float> pointB = {x1, y1};

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

void scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    // Score the intervening grid squares of the ray, to edecrease likelihood of occupancy.
    Point<int> origin;
    Point<int> endpoint;

    origin.x = (int)std::floor(ray.origin.x);
    origin.y = (int)std::floor(ray.origin.y);

    float end_x = ray.origin.x + ray.range*cos(ray.theta);
    float end_y = ray.origin.y + ray.range*sin(ray.theta);
    endpoint.x = (int)std::floor(end_x);
    endpoint.y = (int)std::floor(end_y);

    int dx = abs(endpoint.x - origin.x);
    int dy = abs(endpoint.y - origin.y);
    int sx = origin.x<endpoint.x ? 1 : -1;
    int sy = origin.y<endpoint.y ? 1 : -1;
    int err = dx - dy;
    int x = origin.x;
    int y = origin.y;

    Point<float> current;
    current.x = ray.origin.x;
    current.y = ray.origin.y;

    while (((x != endpoint.x) || (y != endpoint.y)) && (map.isCellInGrid(x, y))) {
        CellOdds l_new = inverse_sensor_model(ray.origin.x, ray.origin.y, ray.theta, current.x, current.y, ray.range);
        CellOdds l_curr = map.logOdds(x, y);
        if (!((l_curr + l_new < -127) || ((l_curr + l_new > 127)))) {
            map.setLogOdds(x, y, l_curr + l_new);
        }

        int e2 = err * 2;
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }
}

void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    MovingLaserScan movingScan(scan, previousPose_, pose, 1);

    for (auto& ray : movingScan) {
        scoreRay(ray, map);
    }

    previousPose_ = pose;
}
