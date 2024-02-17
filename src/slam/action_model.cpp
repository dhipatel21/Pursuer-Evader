#include "action_model.hpp"

ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    rot1 = 0;
    trans = 0;
    rot2 = 0;
    time = 0;

    eps1 = 0;
    eps2 = 0;
    eps3 = 0;

    a1 = 1;
    a2 = 1;
    a3 = 1;
    a4 = 1;

    x_hat = 0;
    y_hat = 0;
    theta_hat = 0;

    std::random_device rd;
    gen = std::mt19937(rd());
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    // We're using the odometry model here
    // !!!!!!!!!!!!!!!!!!!!!!!!!! 
    // Perform straight line to determine k1 k2 ????
    // !!!!!!!!!!!!!!!!!!!!!!!!!

    float x_prime = odometry.x;
    float y_prime = odometry.y;
    float theta_prime = odometry.theta;

    float delta_rot1 = atan2(y_prime - y_hat, x_prime - x_hat) - theta_hat;
    float delta_trans = sqrt(pow(x_prime - x_hat, 2) + pow(y_prime - y_hat, 2));
    float delta_rot2 = theta_prime - theta_hat - delta_rot1;

    // Initialization for sampling normal distributions

    float norm1 = std::normal_distribution<>(0, a1*pow(delta_rot1, 2) + a2*pow(delta_trans, 2))(gen);
    float norm2 = std::normal_distribution<>(0, a3*pow(delta_trans, 2) + a4*pow(delta_rot1, 2) + a4*pow(delta_rot2, 2))(gen);
    float norm3 = std::normal_distribution<>(0, a1*pow(delta_rot2, 2) + a2*pow(delta_trans, 2))(gen);

    rot1 = delta_rot1 - norm1;
    trans = delta_trans - norm2;
    rot2 = delta_rot2 - norm3;

    x_hat = x_prime;
    y_hat = y_prime;
    theta_hat = theta_prime;

    time = odometry.utime;

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
    new_sample.pose.utime = time;

    return new_sample;
}
