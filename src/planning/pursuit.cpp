#include <planning/pursuit.hpp>
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


const float kReachedPositionThreshold = 0.5f;  // must get within this distance of a position for it to be explored

// Define an equality operator for poses to allow direct comparison of two paths
bool operator==(const pose_xyt_t& lhs, const pose_xyt_t& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.theta == rhs.theta);
}

Pursuit::Pursuit(int32_t teamNumber,
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
    
    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Pursuit::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Pursuit::handlePose, this);
    lcmInstance_->subscribe(MESSAGE_CONFIRMATION_CHANNEL, &Pursuit::handleConfirmation, this);
    lcmInstance_->subscribe(PE_REQUEST_CHANNEL, &Pursuit::handleRequest, this);
    lcmInstance_->subscribe(GOOD_MICROPHONE_CHANNEL, &Pursuit::handleGoodMicrophone, this);
    
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


bool Pursuit::pursue()
{
    while((state_ != exploration_status_t::STATE_COMPLETED_EXPLORATION) 
        && (state_ != exploration_status_t::STATE_FAILED_EXPLORATION))
    {
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            runPursuit();
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

void Pursuit::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const occupancy_grid_t* map)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingMap_.fromLCM(*map);
    haveNewMap_ = true;
}


void Pursuit::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingPose_ = *pose;
    haveNewPose_ = true;
}

void Pursuit::handleConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const message_received_t* confirm)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if(confirm->channel == CONTROLLER_PATH_CHANNEL && confirm->creation_time == most_recent_path_time) pathReceived_ = true;
}

void Pursuit::handleRequest(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* request)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    currentTarget_.theta = request->theta;
    currentTarget_.utime = request->utime;
    currentTarget_.x = request->x;
    currentTarget_.y = request->y;
}

void Pursuit::handleGoodMicrophone(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* mic_info)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    evaderInfo_.theta = mic_info->theta;
    evaderInfo_.utime = mic_info->utime;
}

void Pursuit::handleCamera(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* camera_info)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    evaderInfo_.theta = camera_info->theta;
    evaderInfo_.utime = camera_info->utime;
    evaderInfo_.x = camera_info->x;
}

bool Pursuit::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return haveNewMap_ && haveNewPose_;
}


void Pursuit::runPursuit(void)
{
    assert(isReadyToUpdate());
    
    copyDataForUpdate();
    executeStateMachine();
}


void Pursuit::copyDataForUpdate(void)
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
        std::cout << "INFO: Pursuit: Set home pose:" << homePose_.x << ',' << homePose_.y << ',' 
            << homePose_.theta << '\n';
    }
}

void Pursuit::executeStateMachine(void)
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
                nextState = executePursuit(stateChanged);
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

        std::cout << "INFO: Pursuit: A new path was created on this iteration. Sending to Mbot:\n";

        std::cout << "path timestamp: " << currentPath_.utime << "\npath: ";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

        pathReceived_ = false;
        most_recent_path_time = currentPath_.utime;
    }

}

int8_t Pursuit::executeInitializing(void)
{
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to pursuing once the first bit of data has arrived
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    return exploration_status_t::STATE_EXPLORING_MAP;
}

// TODO
int8_t Pursuit::executePursuit(bool initialize)
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

    // If the evader is in range, we win
    if(evaderInfo_.x < CAPTURE_RADIUS)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Else if time is up, we lose
    else if(dt > TRIAL_TIME)
    {
        std::cout << "TIME IS UP; EVADER WINS " << "\n";
        status.status = exploration_status_t::STATUS_FAILED;
    }
    // Else we contine
    else
    {
        std::cout << "Continue pursuit" << std::endl;
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    switch(status.status)
    {
        // Don't change states if we're still a work-in-progress
        case exploration_status_t::STATUS_IN_PROGRESS:
            return exploration_status_t::STATE_EXPLORING_MAP;
            
        // If exploration is completed, then head home
        case exploration_status_t::STATUS_COMPLETE:
            return exploration_status_t::STATE_RETURNING_HOME;
            
        // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
        case exploration_status_t::STATUS_FAILED:
            return exploration_status_t::STATE_FAILED_EXPLORATION;
            
        default:
            std::cerr << "ERROR: Exploration::executeExploringMap: Set an invalid exploration status. Exploration failed!";
            return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}

int8_t Pursuit::executeCompleted(bool initialize)
{
    // Stay in the completed state forever because exploration only explores a single map.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_COMPLETED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);

    // TODO end behavior
    // concludePursuit(true);
    
    return exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

int8_t Pursuit::executeFailed(bool initialize)
{
    // Send the execute failed forever. There is no way to recover.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_FAILED;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);

    // TODO end behavior
    // concludePursuit(false);
    
    return exploration_status_t::STATE_FAILED_EXPLORATION;
}
