#include <planning/pursuit.hpp>
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

    keep_turning = false;
    
    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Pursuit::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Pursuit::handlePose, this);
    lcmInstance_->subscribe(MESSAGE_CONFIRMATION_CHANNEL, &Pursuit::handleConfirmation, this);
    lcmInstance_->subscribe(PE_REQUEST_CHANNEL, &Pursuit::handleRequest, this);
    lcmInstance_->subscribe(GOOD_MICROPHONE_CHANNEL, &Pursuit::handleGoodMicrophone, this);
    lcmInstance_->subscribe(TURN_TO_SOURCE_CHANNEL, &Pursuit::handleTurnToSource, this);

    lcmInstance_->subscribe(CAMERA_1_CHANNEL, &Pursuit::handleCamera, this);
    lcmInstance_->subscribe(CAMERA_2_CHANNEL, &Pursuit::handleCamera, this);
    lcmInstance_->subscribe(CAMERA_3_CHANNEL, &Pursuit::handleCamera, this);
    
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
    evaderInfo_.x = 999;
    evaderInfo_.y = 999;
    evaderInfo_.theta = 0;
    evaderInfo_.utime = 0;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    MotionPlannerParams params;
    params.robotRadius = 0.15;
    planner_.setParams(params);

    start_time = std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::system_clock::now().time_since_epoch()).count();
}


bool Pursuit::pursue()
{
    std::cout << "INFO: Pursue" << std::endl;
    while((state_ != exploration_status_t::STATE_COMPLETED_EXPLORATION) 
        && (state_ != exploration_status_t::STATE_FAILED_EXPLORATION))
    {
        // std::cout << "INFO: Continue pursuit" << std::endl;
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            // std::cout << "INFO: Ready to update" << std::endl;
            runPursuit();
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            // std::cout << "INFO: Not ready to update, sleeping" << std::endl;
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
    std::cout << "INFO: Waypoint request received" << std::endl;
    keep_turning = false;
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
    if (camera_info->y != -1) {
        evaderInfo_.theta = camera_info->theta;
        evaderInfo_.utime = camera_info->utime;
        evaderInfo_.x = camera_info->x;
    }
}

void Pursuit::handleTurnToSource(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* turn_signal)
{
    std::cout << "INFO: Turning request received" << std::endl;
    std::lock_guard<std::mutex> autoLock(dataLock_);
    keep_turning = true;
}

bool Pursuit::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return haveNewMap_ && haveNewPose_;
}

void Pursuit::runPursuit(void)
{
    std::cout << "INFO: Run pursuit" << std::endl;
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
    std::cout << "INFO: Execute state machine" << std::endl;
    bool stateChanged = false;
    int8_t nextState = state_;
    
    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    robot_path_t previousPath = currentPath_;
    
    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        std::cout << "Single state machine iteration" << std::endl;
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
    if(!pathReceived_ && (currentTarget_.utime != 0))
    {
        std::cout << "INFO: Pursuit: The current path was not received by motion_controller, attempting to send again:\n";

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
    std::cout << "INFO: Execute initializing" << std::endl;
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    return exploration_status_t::STATE_EXPLORING_MAP;
}

int8_t Pursuit::executePursuit(bool initialize)
{    
    std::cout << "INFO: Execute pursuit" << std::endl;
    planner_.setMap(currentMap_); // update map from SLAM

    float goalDist = 0;
    if (currentPath_.path_length > 1) {
        goalDist = distance_between_points(Point<float>(currentPose_.x, currentPose_.y), Point<float>(currentPath_.path.back().x, currentPath_.path.back().y));
    }
    else {
        goalDist = std::numeric_limits<float>::max();
    }

    if (!keep_turning && (currentTarget_.utime != 0)) {
        std::cout << "INFO: Begin planning consideration" << std::endl;
        if ((initialize || !planner_.isPathSafe(currentPath_) || goalDist < 2*currentMap_.metersPerCell() || currentPath_.path_length <= 1))
        {
            if(currentMap_.isCellInGrid(currentTarget_.x, currentTarget_.y) // cell is in the grid
                && currentMap_.logOdds(currentTarget_.x, currentTarget_.y) < 0  // cell is unoccupied by an obstacle
                && planner_.isValidGoal(currentTarget_))            // planned pose is within acceptable radius of obstacle
            {
                std::cout << "INFO: Valid goal found, begin planing" << std::endl;
                std::cout << currentTarget_.x << " " << currentTarget_.y << "\n";
                currentPath_ = planner_.planPath(currentPose_, currentTarget_);
            }
            else {
                // otherwise centroid is not suitable, so radial search to find suitable cells on frontier
                pose_xyt_t newPose (currentPose_);
                float radius = 0.02;
                while (radius < 1){
                    std::cout << "radius: " << radius << "\n";
                    for (float angle = 0; angle < 2*M_PI; angle += (M_PI / 8.0)){

                        float dx = radius * cos(angle);
                        float dy = radius * sin(angle);
                        std::cout << dx << " " << dy << "\n";
                        Point<double> coordinate (currentTarget_.x + dx, currentTarget_.y + dy);
                        cell_t cell = global_position_to_grid_cell(coordinate, currentMap_);
                        
                        newPose.x = coordinate.x;
                        newPose.y = coordinate.y;
                        
                        std::cout << newPose.x << " " << newPose.y << "\n";
                        robot_path_t plannedPath;

                        if(currentMap_.isCellInGrid(cell.x, cell.y)             // cell is in the grid
                            && planner_.isValidGoal(newPose))            // planned pose is within acceptable radius of obstacle
                            {
                                std::cout << "INFO: Valid RADIAL goal found, begin planing" << std::endl;
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
    }
    // else if ((currentTarget_.utime != 0)) {
    else if (true) {
        std::cout << "INFO: Turning until camera located" << std::endl;
        pose_xyt_t turn;
        turn = currentPose_;
        turn.theta += 0.5;

        robot_path_t turn_path;
        turn_path.path.push_back(turn);
        turn_path.path.push_back(turn);
        turn_path.path_length = 2;
        turn_path.utime = 0;
        
        currentPath_ = turn_path;
    }
    else {
        std::cout << "WARNING: No target information has been received." << std::endl;
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


    // If the evader is in range, we win
    if(evaderInfo_.x < CAPTURE_RADIUS)
    {
        std::cout << "INFO: EVADER CAPUTRED " << "\n";
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Else if time is up, we lose
    else if(dt > TRIAL_TIME)
    {
        std::cout << "INFO: TIME IS UP; EVADER WINS " << "\n";
        status.status = exploration_status_t::STATUS_FAILED;
    }
    // Else we contine
    else
    {
        std::cout << "INFO: Continue pursuit" << std::endl;
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
    mbot_motor_command_t cmd = {0, 0, 0};
    lcmInstance_->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
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
    mbot_motor_command_t cmd = {0, 0, 0};
    lcmInstance_->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    return exploration_status_t::STATE_FAILED_EXPLORATION;
}
