#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    float rot1 = atan2(yprime_bar - y, xprime_bar - x) - theta_bar;
    float trans = sqrt((xprime_bar - x) * (xprime_bar - x) + (yprime_bar - y) * (yprime_bar - y));
    float rot2 = thetaprime_bar - theta_bar - rot1;

    float rot1prime = ;
    float transprime = ;
    float rot2prime = ;

    float xprime = x + transprime * cos(theta + rot1prime);
    float yprime = y + transprime * sin(theta + rot1prime);
    float thetaprime = theta + rot1prime + rot2prime;

    return sample;
}
