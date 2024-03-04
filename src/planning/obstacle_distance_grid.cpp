#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>

#include <queue>

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.

    // BRUSHFIRE ALGORITHM

    std::queue<std::pair<int, int>> Queue;
    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};

    // Set distance to all obstacles to 0

    for (int x = 0; x < width_; ++x){ // Loop through all cells in OccupancyGrid
        for (int y = 0; y < height_; ++y){
            cells_.push_back(-1);
            
            if (map.logOdds(x, y) > 0){ // if obstacle at x, y
                int index = cellIndex(x, y);
                distance(x, y) = 0;
                Queue.push({x, y});
            }

        }
    }

    // Wavefront propogation
    while (!Queue.empty()){
        std::pair<int, int> coords = Queue.front();
        int x = coords.first;
        int y = coords.second;
        Queue.pop();

        // Explore neighbors
        for (int i = 0; i < 4; ++i){
            int newX= x + dr[i];
            int newY = y + dc[i];

            // If cell is in grid and unvisited
            if (isCellInGrid(newX, newY) 
                && distance(newX, newY) == -1) {
                distance(newX, newY) = distance(x, y) + 1;
            }
        }
    }
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
