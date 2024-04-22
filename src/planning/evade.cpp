#include <planning/evade.hpp>
#include <planning/frontiers.hpp>
#include <planning/planning_channels.h>
#include <planning/pe_channels.h>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <unistd.h>
#include <cassert>
#include <chrono>

const float kReachedPositionThreshold = 0.5f;  // must get within this distance of a position for it to be explored

// Define an equality operator for poses to allow direct comparison of two paths
bool operator==(const pose_xyt_t& lhs, const pose_xyt_t& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.theta == rhs.theta);
}

Evade::Evade(int32_t teamNumber,
                         lcm::LCM* lcmInstance)
: teamNumber_(teamNumber)
, state_(exploration_status_t::STATE_INITIALIZING)
, haveNewPose_(false)
, haveNewMap_(false)
, haveHomePose_(false)
, lcmInstance_(lcmInstance)
, pathReceived_(false)
{
    assert(lcmInstance_);   // confirm a nullptr wasn't passed in
    
    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Evade::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Evade::handlePose, this);
    lcmInstance_->subscribe(MESSAGE_CONFIRMATION_CHANNEL, &Evade::handleConfirmation, this);
    lcmInstance_->subscribe(PE_REQUEST_CHANNEL, &Evade::handleRequest, this);
    lcmInstance_->subscribe(PE_SHUTDOWN_CHANNEL, &Evade::handleShutdown, this);
    
    // Send an initial message indicating that the exploration module is initializing. Once the first map and pose are
    // received, then it will change to the exploring map state.
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_IN_PROGRESS;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    MotionPlannerParams params;
    params.robotRadius = 0.15;
    planner_.setParams(params);

    start_time = std::chrono::system_clock::now();
}


bool Evade::evade()
{
    while((state_ != exploration_status_t::STATE_COMPLETED_EXPLORATION) 
        && (state_ != exploration_status_t::STATE_FAILED_EXPLORATION))
    {
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            runEvade();
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            usleep(10000);
        }
    }
    
    // If the state is completed, then we didn't fail
    return state_ == exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

void Evade::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const occupancy_grid_t* map)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingMap_.fromLCM(*map);
    haveNewMap_ = true;
}


void Evade::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingPose_ = *pose;
    haveNewPose_ = true;
}

void Evade::handleConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const message_received_t* confirm)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if(confirm->channel == CONTROLLER_PATH_CHANNEL && confirm->creation_time == most_recent_path_time) pathReceived_ = true;
}

void Evade::handleRequest(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* request)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    currentTarget_.theta = request->theta;
    currentTarget_.utime = request->utime;
    currentTarget_.x = request->x;
    currentTarget_.y = request->y;
}

// TODO
void Evade::handleShutdown(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const exploration_status_t* request)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    while(1) {
        std::cout << "SHUTDOWN MESSAGE RECEIVED, SHUTTING DOWN NOW" << std::endl;
        concludeEvasion(false);
    }
}

bool Evade::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return haveNewMap_ && haveNewPose_;
}


void Evade::runEvade(void)
{
    assert(isReadyToUpdate());
    
    copyDataForUpdate();
    executeStateMachine();
}


void Evade::copyDataForUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    
    // Only copy the map if a new one has arrived because it is a costly operation
    if(haveNewMap_)
    {
        currentMap_ = incomingMap_;
        haveNewMap_ = false;
    }
    
    // Always copy the pose because it is a cheap copy
    currentPose_ = incomingPose_;
    haveNewPose_ = false;
    
    // The first pose received is considered to be the home pose
    if(!haveHomePose_)
    {
        homePose_ = incomingPose_;
        haveHomePose_ = true;
        std::cout << "INFO: Evade: Set home pose:" << homePose_.x << ',' << homePose_.y << ',' 
            << homePose_.theta << '\n';
    }
}

void Evade::executeStateMachine(void)
{
    bool stateChanged = false;
    int8_t nextState = state_;
    
    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    robot_path_t previousPath = currentPath_;
    
    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        switch(state_)
        {
            case exploration_status_t::STATE_INITIALIZING:
                nextState = executeInitializing();
                break;

            case exploration_status_t::STATE_EXPLORING_MAP:
                nextState = executeEvade(stateChanged);
                break;

            case exploration_status_t::STATE_COMPLETED_EXPLORATION:
                nextState = executeCompleted(stateChanged);
                break;
                
            case exploration_status_t::STATE_FAILED_EXPLORATION:
                nextState = executeFailed(stateChanged);
                break;
        }
        
        stateChanged = nextState != state_;
        state_ = nextState;
        
    } while(stateChanged);

    //if path confirmation was not received, resend path
    if(!pathReceived_)
    {
        std::cout << "the current path was not received by motion_controller, attempting to send again:\n";

        std::cout << "timestamp: " << currentPath_.utime << "\n";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
    }

    //if path changed, send current path
    if(previousPath.path != currentPath_.path)
    { 

        std::cout << "INFO: Evade: A new path was created on this iteration. Sending to Mbot:\n";

        std::cout << "path timestamp: " << currentPath_.utime << "\npath: ";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

        pathReceived_ = false;
        most_recent_path_time = currentPath_.utime;
    }

}

int8_t Evade::executeInitializing(void)
{
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to evading once the first bit of data has arrived
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    return exploration_status_t::STATE_EXPLORING_MAP;
}

int8_t Evade::executeEvade(bool initialize)
{
    planner_.setMap(currentMap_); // update map from SLAM

    /////////////////////////////// End student code ///////////////////////////////

    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;

    const auto p1 = std::chrono::system_clock::now();
    const auto dt = std::chrono::duration_cast<std::chrono::seconds>(
                   p1.time_since_epoch()).count();
    
    // If time has expired, we win
    if (dt > TRIAL_TIME)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Else we should keep running (we let the shutdown handler tell us we lose)
    else
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    switch(status.status)
    {
        // Don't change states if we're still a work-in-progress
        case exploration_status_t::STATUS_IN_PROGRESS:
            return exploration_status_t::STATE_EXPLORING_MAP;
            
        // If evasion is completed, shut down gracefully
        case exploration_status_t::STATUS_COMPLETE:
            return exploration_status_t::STATE_RETURNING_HOME;
            
        default:
            std::cerr << "ERROR: Evade::executeEvade: Set an invalid exploration status. Evasion failed!";
            return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}

int8_t Evade::executeCompleted(bool initialize)
{
    // Stay in the completed state forever because exploration only explores a single map.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_COMPLETED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);

    concludeEvasion(true);
    
    return exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

int8_t Evade::executeFailed(bool initialize)
{
    // Send the execute failed forever. There is no way to recover.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_FAILED;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);

    concludeEvasion(false);
    
    return exploration_status_t::STATE_FAILED_EXPLORATION;
}

// TODO
void Evade::concludeEvasion(bool victory) 
{
    return;
}