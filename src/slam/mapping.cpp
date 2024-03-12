#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <limits>

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance), kHitOdds_(hitOdds), kMissOdds_(missOdds), initialized_(false)
{
}


// CellOdds Mapping::inverse_sensor_model(float x, float y, float theta, float x1, float y1, float range) {
//     Point<float> pointA = {x, y};
//     Point<float> pointB = {x1, y1};

//     float r = distance_between_points(pointA, pointB);
//     float phi = atan2(pointB.y - pointA.y, pointB.x - pointA.x) - theta;
//     float alpha = 0.1; // dimension of cell is 10x10 cm
//     float z_max = 10; // max allowable distance is 10 m (???? Might need to confirm this)

//     // if the distance to the cell is greater than the maximum allowable range or the measured range
//     // then it is beyond an obstacle and/or unknown
//     if (r > std::min(z_max, range + alpha/2) || ) { 
//         return 0;
//     }
//     // if the measured range to obstacle is less than the maximum allowable range AND the difference
//     // between the measured range and the distance to that cell is ~0, then the cell is an obstacle
//     else if ((range < z_max) && (abs(r - range) < alpha/2)) { 
//         return kHitOdds_;
//     }
//     // if distance to cell is less than measured range to obstacle, then it is unoccupied
//     else if (r <= range) {
//         return -kMissOdds_;
//     }
//     // return unkown if we don't hit any of the other cases 
//     else {
//         return 0;
//     }
// }

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    // Score the intervening grid squares of the ray, to edecrease likelihood of occupancy.
    // std::cout << "score ray\n";

    Point<int> origin;
    Point<int> endpoint;

    Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
    Point<double> endpointPos;

    endpointPos.x = ray.range * std::cos(ray.theta) + ray.origin.x;
    endpointPos.y = ray.range * std::sin(ray.theta) + ray.origin.y;
    endpoint = global_position_to_grid_cell(endpointPos, map);

    int dx = abs(endpoint.x - rayStart.x);
    int dy = abs(endpoint.y - rayStart.y);
    int sx = rayStart.x < endpoint.x ? 1 : -1;
    int sy = rayStart.y < endpoint.y ? 1 : -1;
    int err = dx - dy;
    int x = rayStart.x;
    int y = rayStart.y;


    // std::cout << "origin x" << rayStart.x << " y " << rayStart.y << "\n\n";

    // std::cout << "endpoint x" << endpoint.x << " y " << endpoint.y << "\n\n";

    while (((x != endpoint.x) || (y != endpoint.y)) && (map.isCellInGrid(x, y))) {
        // std::cout << "x " << x << " y " << y << '\n';
        // std::cout << "min odds " << std::numeric_limits<CellOdds>::min() << " max odds " << std::numeric_limits<CellOdds>::max() << '\n';
        if (map(x, y) - kMissOdds_ > std::numeric_limits<CellOdds>::min()) {
            decreaseCellOdds(x, y, map);
        }
        else {
            map(x, y) = std::numeric_limits<CellOdds>::min();
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

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map) {
    // std::cout << "score endpoint\n";

    if (ray.range <= kMaxLaserDistance_) {
        // Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<double> rayEnd;

        rayEnd.x = ray.range * std::cos(ray.theta) + ray.origin.x;
        rayEnd.y = ray.range * std::sin(ray.theta) + ray.origin.y;
        Point<int> rayCell = global_position_to_grid_cell(rayEnd, map);

        if (map.isCellInGrid(rayCell.x, rayCell.y)) {
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}

void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    // std::cout << "update map" << '\n';
    if (!initialized_) {
        previousPose_ = pose;
    }
    MovingLaserScan movingScan(scan, previousPose_, pose);

    for (auto& ray : movingScan) {
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }
    initialized_ = true;
    previousPose_ = pose;
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map) {
    if (map(x, y) - kMissOdds_  > std::numeric_limits<CellOdds>::min()) {
        map(x, y) -= kMissOdds_;
    }
    else {
        map(x, y) = std::numeric_limits<CellOdds>::min();
    }
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map) {
    if (map(x, y) + kHitOdds_ < std::numeric_limits<CellOdds>::max()) {
        map(x, y) += kHitOdds_;
    }
    else {
        map(x, y) = std::numeric_limits<CellOdds>::max();
    }
}
