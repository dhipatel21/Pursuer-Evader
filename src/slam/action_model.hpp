#ifndef SLAM_ACTION_MODEL_HPP
#define SLAM_ACTION_MODEL_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <random>
#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

struct particle_t;

/**
* ActionModel implements the sampling-based odometry action model for estimating the motion of the robot between
* time t and t'.
* 
* An action model is used to propagate a sample from the prior distribution, x, into
* the proposal distribution, x', based on the supplied motion estimate of the robot
* in the time interval [t, t'].
*
* To use the ActionModel, a two methods exist:
*
*   - bool updateAction(const pose_xyt_t& odometry);
*   - particle_t applyAction(const particle_t& sample);
*
* updateAction() provides the most recent odometry data so the action model can update the distributions from
* which it will sample.
*
* applyAction() applies the action to the provided sample and returns a new sample that can be part of the proposal 
* distribution for the particle filter.
*/
class ActionModel
{
public:
    
    /**
    * Constructor for ActionModel.
    */
    ActionModel(void);
    
    /**
     * // TLDR: call this every time you get more odometry data to keep action up to date.
     * 
    * updateAction sets up the motion model for the current update for the localization.
    * After initialization, calls to applyAction() will be made, so all distributions based on sensor data
    * should be created here.
    *
    * \param    odometry            Current odometry data from the robot
    * \return   The pose transform distribution representing the uncertainty of the robot's motion.
    */
    bool updateAction(const pose_xyt_t& odometry);
    
    /**
     * // TLDR: call this every time you actually want to generate a new action sample and realize the updates made.
     * 
    * applyAction applies the motion to the provided sample and returns a new sample that
    * can be part of the proposal distribution for the particle filter.
    *
    * \param    sample          Sample to be moved
    * \return   New sample based on distribution from the motion model at the current update.
    */
    particle_t applyAction(const particle_t& sample);
    
private:
    
    ////////// TODO: Add private member variables needed for you implementation ///////////////////
    float rot1;  // alpha or first rotation
    float trans; // delta s or translation
    float rot2;  // delta theta minus alpha or second rotation
    float time; // current time of sample

    // samples from normal distribution, representing errors
    float eps1;
    float eps2;
    float eps3;

    // a-values
    float a1; // k1
    float a2; // cross correlation
    float a3; // k3
    float a4; // cross correlation

    // previous state
    float x_hat;
    float y_hat;
    float theta_hat;

    // rng
    std::random_device rd;
};

#endif // SLAM_ACTION_MODEL_HPP
