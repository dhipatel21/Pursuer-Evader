#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

void ObstacleDistanceGrid::initializeDistances(const OccupancyGrid& map)
{
    int width = map.widthInCells();
    int height = map.heightInCells();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (map.logOdds(x, y) < 0) {
                distance(x, y) = MAXFLOAT;
            }
            else {
                distance(x, y) = 0.0;
            }
        }
    }
}

bool ObstacleDistanceGrid::is_cell_free(cell_t cell, const OccupancyGrid& map)
{
    return map.logOdds(cell.x, cell.y) <= 0;
}

bool ObstacleDistanceGrid::is_cell_occupied(cell_t cell, const OccupancyGrid& map)
{
    return map.logOdds(cell.x, cell.y) > 0;
}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    initializeDistances(map);

    std::priority_queue<DistanceNode> searchQueue;
    enqueue_obstacle_cells(map, searchQueue);

    while (!(searchQueue.empty())) {
        auto nextNode = searchQueue.top();
        searchQueue.pop();
        expand_node(nextNode, searchQueue);
    }

    int max = -100;
    for (int i = 0; i < cells_.size(); i++) {
        if (cells_[i] > max) {
            max = cells_[i];
        }
    }
    if (max == -1) {
        for (int i = 0; i < cells_.size(); i++) {
            cells_[i] = MAXFLOAT;
        }
    }
}

void ObstacleDistanceGrid::enqueue_obstacle_cells(const OccupancyGrid& map, std::priority_queue<DistanceNode>& search_queue) {
    int width = map.widthInCells();
    int height = map.heightInCells();
    cell_t cell;

    for(cell.y = 0; cell.y < height; ++cell.y) {
        for(cell.x = 0; cell.x < width; ++cell.x) {
            if (is_cell_occupied(cell, map)) {
                expand_node(DistanceNode(cell, 0), search_queue);
            }
        }
    }
}

void ObstacleDistanceGrid::expand_node(DistanceNode node, std::priority_queue<DistanceNode>& search_queue) {
    // 8-way expansion
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    for (int n = 0; n < 8; ++n) {
        cell_t adjacentCell(node.cell.x + xDeltas[n], node.cell.y + yDeltas[n]);
        if (isCellInGrid(adjacentCell.x, adjacentCell.y)) {
            if(cells_[cellIndex(adjacentCell.x, adjacentCell.y)] == MAXFLOAT) {
                DistanceNode adjacentNode(adjacentCell, node.distance+1);
                cells_[cellIndex(adjacentCell.x, adjacentCell.y)] = adjacentNode.distance * metersPerCell();
                search_queue.push(adjacentNode);
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
