#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
   float likelihood = 0.0;
   // Iterate over laser scan readings
    for (size_t i = 0; i < scan.num_ranges; ++i) {
        // Get current range measurement
        double range = scan.ranges[i];

        // Get angle of current laser beam
        double angle = scan.start_angle + i * scan.angle_increment;

        // Compute endpoint of the laser beam in the map frame
        Point<float> endpoint;
        endpoint.x = sample.pose.x + range * cos(angle + sample.pose.theta);
        endpoint.y = sample.pose.y + range * sin(angle + sample.pose.theta);

    }

    return log_likelihood;
}
