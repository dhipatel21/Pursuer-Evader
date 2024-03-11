#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <chrono>

using namespace std::chrono;


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
   
    PriorityQueue open;

    Point<double> start_point(start.x, start.y);
    cell_t start_cell = global_position_to_grid_cell(start_point, distances);
    Point<double> goal_point(goal.x, goal.y);
    cell_t goal_cell = global_position_to_grid_cell(goal_point, distances);

    Node start_node(start_cell.x, start_cell.y);
    Node goal_node(goal_cell.x, goal_cell.y);

    PriorityQueue closed;


}

double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances) {
    int dx = std::abs(goal->cell.x - from->cell.x);
    int dy = std::abs(goal->cell.y - from->cell.y);
    double diag_distance = 1.414;

    double h_cost = (dx+dy) + (diag_distance - 2) * std::min(dx,dy);
    return h_cost;
}

double g_cost(Node* start, Node* current, const ObstacleDistanceGrid& distances, const SearchParams& params) {
    int dx = std::abs(start->cell.x - current->cell.x);
    int dy = std::abs(start->cell.y - current->cell.y);
    double diag_distance = 1.414;

    double g_cost = (dx+dy) + (diag_distance - 2) * std::min(dx,dy);
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid* distances, const SearchParams& params) {
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    std::vector<Node*> children;
    for (int n=0; n<8; ++n) {
        int cell_x = node->cell.x + xDeltas[n];
        int cell_y = node->cell.y + yDeltas[n];
        Node* childNode = new Node(cell_x, cell_y);

        if(!distances->isCellInGrid(cell_x, cell_y))
            continue;
        
        if((*distances)(cell_x, cell_y) <= params.minDistanceToObstacle)
            continue;

        children.push_back(childNode);
    }
    return children;
}

std::vector<pose_xyt_t> make_path(Node* goal_node, Node* start_node, const ObstacleDistanceGrid& distances) {
    // Full stack extract path
    std::vector<Node*> npath = extract_node_path(goal_node, start_node);
    npath = prune_node_path(npath);
    std::vector<pose_xyt_t> ppath = extract_pose_path(npath, distances);

    return ppath;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node) {
    // Returns: Path to goal
}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath) {
    // Returns: Smoothed path to goal (via midpoint pruning)
}

std::vector<pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances) {
    // Returns: Pose path
}

bool is_in_list(Node* node, std::vector<Node*> list) {
    for (auto &&item : list) {
        if(*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list) {
    for (auto &&item : list) {
        if(*node == *item) return item;
    }
    return NULL;
}