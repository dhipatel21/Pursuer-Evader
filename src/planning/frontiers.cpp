#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <common/grid_utils.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>
#include <cmath>

bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
robot_path_t path_to_frontier(const frontier_t& frontier, 
                              const pose_xyt_t& pose, 
                              const OccupancyGrid& map,
                              const MotionPlanner& planner);
pose_xyt_t nearest_navigable_cell(pose_xyt_t pose, 
                                  Point<float> desiredPosition, 
                                  const OccupancyGrid& map,
                                  const MotionPlanner& planner);
pose_xyt_t search_to_nearest_free_space(Point<float> position, const OccupancyGrid& map, const MotionPlanner& planner);
double path_length(const robot_path_t& path);


std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map, 
                                           const pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;
    
    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);
  
    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            
            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);
                
                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontiers;
}


robot_path_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers, 
                                   const pose_xyt_t& robotPose,
                                   const OccupancyGrid& map,
                                   const MotionPlanner& planner)
{
    ///////////// TODO: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */
    robot_path_t plannedPath;
    
    // find and sort centroids by distance
    std::vector<Point<float>> centroids;

    for (auto &&frontier : frontiers) {
        Point<float> centroid = find_frontier_centroid(frontier);

        centroids.push_back(centroid);
    }

    std::sort(centroids.begin(), centroids.end(), sort_by_distance);

    // check for path to each centroid
    for (auto &&centroid : centroids){
        cell_t centroidCell = global_position_to_grid_cell(centroid, map);
        pose_xyt_t newPose (robotPose);

        newPose.x = centroidCell.x;
        newPose.y = centroidCell.y;

        // if cell is suitable goal pose, we're good
        if(map.isCellInGrid(centroidCell.x, centroidCell.y) // cell is in the grid
        && map.logOdds(centroidCell.x, centroidCell.y) < 0  // cell is unoccupied by an obstacle
        && planner.isValidGoal(newPose))            // planned pose is within acceptable radius of obstacle
        {
            plannedPath = planner.planPath(robotPose, newPose);

            if (planner.isPathSafe(plannedPath)){   // is path still safe
                return plannedPath;
            }
        }

        // otherwise centroid is not suitable, so radial search to find suitable cells on frontier
        int radius = 1;
        while (radius < 10){
            for (float angle = 0; angle < 360; angle += 22.5){
                float dx = radius * cos(angle);
                float dy = radius * sin(angle);
                
                Point<double> coordinate (centroid.x + dx, centroid.y + dy);
                cell_t cell = global_position_to_grid_cell(coordinate, map);

                if(is_frontier_cell(cell.x, cell.y, map)    // cell is on the frontier
                && map.isCellInGrid(cell.x, cell.y)         // cell is in the grid
                && map.logOdds(cell.x, cell.y) < 0          // cell is unoccupied by an obstacle
                && planner.isValidGoal(newPose))            // planned pose is within acceptable radius of obstacle
                    {
                        plannedPath = planner.planPath(robotPose, newPose);

                        if (planner.isPathSafe(plannedPath)){   // is path still safe
                            return plannedPath;
                        }
                    }
                
                ++radius;
            }
        }
    }

}

bool sort_by_distance(Point<float>& centroid1, Point<float>& centroid2, const pose_xyt_t& robotPose){
    return distance_from_robot(centroid1, robotPose) < distance_from_robot(centroid2, robotPose);
}

double distance_from_robot(Point<float> point, const pose_xyt_t& robotPose) {
    return std::sqrt(std::pow((point.x - robotPose.x), 2) + std::pow((point.y - robotPose.y), 2));
}

Point<float> find_frontier_centroid(frontier_t frontier)
{
    float sumX = 0;
    float sumY = 0;

    for (auto &&coord : frontier.cells){
        sumX += coord.x;
        sumY += coord.y;
    }

    float centroidX = sumX / frontier.cells.size();
    float centroidY = sumY / frontier.cells.size();

    Point<float> centroid (centroidX, centroidY);

    auto const it = std::lower_bound(frontier.cells.begin(), frontier.cells.end(), centroid);

    return *it;
}

bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell if a frontier if it has log-odds 0 and a neighbor has log-odds < 0
    
    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }
    
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }
    
    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);
    
    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };
 
    frontier_t frontier;
    
    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end()) 
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontier;
}