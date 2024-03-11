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
    
    // robot_path_t path;
    // path.utime = start.utime;
    // path.path.push_back(start);    
    // path.path_length = path.path.size();
    // return path;
    std::priority_queue<Node*, std::vector<Node*>, Compare_Node> open;
    std::vector<Node*> closed;

    // Create starting node and set its f_cost
    Node* startNode = Node(start.x, start.y);
    startNode->f_cost = 0.0;
    startNode->g_cost = 0.0;

    // Add starting node to open list
    open.push(startNode);

    while (!open.empty()) {
        // Get the node with the lowest f_cost from the open list
        Node* currentNode = open.top();
        open.pop();

        // Check if current node is the goal
        if (currentNode->cell.x == goal.x && currentNode->cell.y == goal.y) {
            // Extract path from goal node and return
            return extract_pose_path(extract_node_path(currentNode, startNode), distances);
        }

        // Move current node to closed list
        closed.push_back(currentNode);

        // Expand current node and get its children
        std::vector<Node*> kids = expand_node(currentNode, &distances, params);

        // Loop through children
        for (Node* kid : kids) {
            // Calculate g_cost from current node to child
            double cost = g_cost(currentNode, child, distances, params);

            // Check if child is already in closed list and has lower g_cost in closed list
            if (is_in_list(kid, closed) && get_from_list(kid, closed)->g_cost <= cost) {
                continue;
            }

            // Check if child is already in open list
            if (is_in_list(kid, open)) {
                // Update g_cost and parent if new path is better
                Node* existingNode = get_from_list(kid, open);
                if (cost < existingNode->g_cost) {
                    existingNode->parent = currentNode;
                    existingNode->g_cost = cost;
                    existingNode->f_cost = g_cost(currentNode, kid, distances, params) + h_cost(kid, Node(goal.x, goal.y), distances);
                }
            } else {
                // Set child's parent, g_cost, and f_cost
                kid->parent = currentNode;
                kid->g_cost = cost;
                kid->f_cost = g_cost(currentNode, kid, distances, params) + h_cost(kid, Node(goal.x, goal.y), distances);

                // Add child to open list
                open.push(kid);
            }
        }
    }
    return NULL;
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
    std::vector<Node*> path;
    Node* currentNode = goal_node;
    while (currentNode != start_node) {
        path.push_back(currentNode);
        currentNode = currentNode->parent;
    }

    path.push_back(start_node);
    return path;
}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath) {
    // TODO:
}

std::vector<pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances) {
    std::vector<pose_xyt_t> path;
    for (auto node : nodes) {
        pose_xyt_t pose;
        pose.x = node->cell.x;
        pose.y = node->cell.y;
        // pose.theta = ;
        path.push_back(pose);
    }
    return path;
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