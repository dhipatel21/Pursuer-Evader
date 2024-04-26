#include <planning/evade.hpp>
#include <planning/frontiers.hpp>
#include <planning/planning_channels.h>
#include <planning/pe_channels.h>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
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
    currentTarget_.x = 0;
    currentTarget_.y = 0;
    currentTarget_.theta = 0;
    currentTarget_.utime = 0;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    MotionPlannerParams params;
    params.robotRadius = 0.15;
    planner_.setParams(params);

    start_time = std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::system_clock::now().time_since_epoch()).count();
}


bool Evade::evade()
{
    std::cout << "INFO: Evade" << std::endl;
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
    std::cout << "INFO: Waypoint request received" << std::endl;
    std::lock_guard<std::mutex> autoLock(dataLock_);
    currentTarget_.theta = request->theta;
    currentTarget_.utime = request->utime;
    currentTarget_.x = request->x;
    currentTarget_.y = request->y;
}

void Evade::handleShutdown(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const exploration_status_t* request)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    while(1) {
        std::cout << "SHUTDOWN MESSAGE RECEIVED, SHUTTING DOWN NOW" << std::endl;
        executeFailed(true);
    }
}

bool Evade::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return haveNewMap_ && haveNewPose_;
}


void Evade::runEvade(void)
{
    std::cout << "INFO: Run evade" << std::endl;
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
            << homePose_.theta  << std::endl;
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
        std::cout << "INFO: Evade: The current path was not received by motion_controller, attempting to send again:"  << std::endl;

        std::cout << "timestamp: " << currentPath_.utime  << std::endl;

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); " << std::endl;
        }

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
    }

    //if path changed, send current path
    if(previousPath.path != currentPath_.path)
    { 

        std::cout << "INFO: Evade: A new path was created on this iteration. Sending to Mbot:" << std::endl;

        std::cout << "path timestamp: " << currentPath_.utime << "\npath: " << std::endl;

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); " << std::endl;
        }

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

        pathReceived_ = false;
        most_recent_path_time = currentPath_.utime;
    }

}

int8_t Evade::executeInitializing(void)
{
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to evading once the first bit of data has arrived
    std::cout << "INFO: Execute initializing" << std::endl;
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
    std::cout << "INFO: Execute evasion" << std::endl;
    planner_.setMap(currentMap_); // update map from SLAM

    float goalDist = 0;
    if (currentPath_.path_length > 1) {
        goalDist = distance_between_points(Point<float>(currentPose_.x, currentPose_.y), Point<float>(currentPath_.path.back().x, currentPath_.path.back().y));
    }
    else {
        goalDist = std::numeric_limits<float>::max();
    }

    if ((initialize || !planner_.isPathSafe(currentPath_) || goalDist < 2*currentMap_.metersPerCell() || currentPath_.path_length <= 1))
    {
        if(currentMap_.isCellInGrid(currentTarget_.x, currentTarget_.y) // cell is in the grid
            && currentMap_.logOdds(currentTarget_.x, currentTarget_.y) < 0  // cell is unoccupied by an obstacle
            && planner_.isValidGoal(currentTarget_))            // planned pose is within acceptable radius of obstacle
        {
            std::cout << "INFO: Valid goal found, begin planing" << std::endl;
            std::cout << currentTarget_.x << " " << currentTarget_.y  << std::endl;
            currentPath_ = planner_.planPath(currentPose_, currentTarget_);
        }
        else {
            // otherwise centroid is not suitable, so radial search to find suitable cells on frontier
            pose_xyt_t newPose (currentPose_);
            float radius = 0.02;
            while (radius < 1){
                std::cout << "radius: " << radius  << std::endl;
                for (float angle = 0; angle < 2*M_PI; angle += (M_PI / 8.0)){

                    float dx = radius * cos(angle);
                    float dy = radius * sin(angle);
                    std::cout << dx << " " << dy  << std::endl;
                    Point<double> coordinate (currentTarget_.x + dx, currentTarget_.y + dy);
                    cell_t cell = global_position_to_grid_cell(coordinate, currentMap_);
                    
                    newPose.x = coordinate.x;
                    newPose.y = coordinate.y;
                    
                    std::cout << newPose.x << " " << newPose.y  << std::endl;
                    robot_path_t plannedPath;

                    if(currentMap_.isCellInGrid(cell.x, cell.y)             // cell is in the grid
                        && planner_.isValidGoal(newPose))            // planned pose is within acceptable radius of obstacle
                        {
                            plannedPath = planner_.planPath(currentPose_, newPose);
                            if (planner_.isPathSafe(plannedPath)){   // is path still safe
                                currentPath_ = plannedPath;
                            }
                        }
                }
                radius += 0.02;
            }
        }
    }

    /////////////////////////////// End student code ///////////////////////////////

    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;

    const auto p1 = std::chrono::system_clock::now();
    const int current_time = std::chrono::duration_cast<std::chrono::seconds>(
                   p1.time_since_epoch()).count();
    
    int64_t dt = current_time - start_time;
    std::cout << "Current dt: " << dt << std::endl;

    // If time has expired, we win
    if (dt > TRIAL_TIME)
    {
        std::cout << "INFO: TIME IS UP; EVADER WINS "  << std::endl;
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Else we should keep running (we let the shutdown handler tell us we lose)
    else
    {
        std::cout << "INFO: Continue evasion" << std::endl;
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
        
        // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
        case exploration_status_t::STATUS_FAILED:
            return exploration_status_t::STATE_FAILED_EXPLORATION;

        default:
            std::cerr << "ERROR: Evade::executeEvade: Set an invalid exploration status. Evasion failed!" << std::endl;
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

    // TODO end behavior
    mbot_motor_command_t cmd = {0, 0, 0};
    lcmInstance_->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
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

    // TODO end behavior
    mbot_motor_command_t cmd = {0, 0, 0};
    lcmInstance_->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    return exploration_status_t::STATE_FAILED_EXPLORATION;
}
