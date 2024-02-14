#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    rot1 = 0;
    trans = 0;
    rot2 = 0;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    // !!!!!!!!!!!!!!!!!!!!!!!!!! 
    // Perform straight line to determine k1 k2 ????
    // !!!!!!!!!!!!!!!!!!!!!!!!!

    float k1 = 1;
    float k2 = 1;

    float delta_x = odometry.x;
    float delta_y = odometry.y;
    float delta_theta = odometry.theta;

    float delta_rot1 = atan2(delta_y, delta_x);
    float delta_trans = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    float delta_rot2 = delta_theta - delta_rot1;

    // Initialization for sampling normal distributions
    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<float> norm1(0, k1*abs(rot1));
    std::normal_distribution<float> norm2(0, k2*abs(trans));
    std::normal_distribution<float> norm3(0, k1*abs(rot2));

    rot1 = rot1 - norm1(gen);
    trans = trans - norm2(gen);
    rot2 = rot2 - norm3(gen);

    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t new_sample;
    new_sample.parent_pose = sample.pose;

    float x_0 = new_sample.parent_pose.x;
    float y_0 = new_sample.parent_pose.y;
    float theta_0 = new_sample.parent_pose.theta;

    new_sample.pose.x = x_0 + (trans * cos(theta_0 + rot1));
    new_sample.pose.y = y_0 + (trans * sin(theta_0 + rot1));
    new_sample.pose.theta = theta_0 + rot1 + rot2;

    return new_sample;
}
