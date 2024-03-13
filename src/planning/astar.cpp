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

    auto start_time = high_resolution_clock::now();
    // path.path_length = path.path.size();
    // return path;
    PriorityQueue open;

    Point<double> start_point(double(start.x), double(start.y));
    Point<double> goal_point(double(goal.x), double(goal.y));

    Point<int> start_pos = global_position_to_grid_cell(start_point, distances);
    Point<int> goal_pos = global_position_to_grid_cell(goal_point, distances);

    std::cout << "start pos in grid x " << start_pos.x << " y " << start_pos.y << std::endl;
    std::cout << "goal pos in grid x " << start_pos.x << " y " << start_pos.y << std::endl;

    Node* startNode = new Node(start_pos.x, start_pos.y);
    startNode->g_cost = g_cost(startNode, startNode, distances, params);
    startNode->h_cost = h_cost(startNode, startNode, distances);

    Node* goalNode = new Node(goal_pos.x, goal_pos.y);
    goalNode->g_cost = g_cost(startNode, goalNode, distances, params);
    goalNode->h_cost = h_cost(goalNode, goalNode, distances);

    std::vector<Node*> closed;

    if (startNode->cell.x == goalNode->cell.x && startNode->cell.y == goalNode->cell.y) {
        path.path = make_path(startNode, startNode, distances, goalNode);

        for (auto node : closed) {
            delete node;
        }
        delete goalNode;

        path.path_length = path.path.size();
        return path;
    }

    open.push(startNode);

    std::cout << "begin planning" << std::endl;
    while (!open.empty()) {
        auto current_time = high_resolution_clock::now();
        std::chrono::microseconds dt = duration_cast<microseconds>(current_time - start_time);

        if (dt.count() > CUTOFF_US) {
            std::cout << "TIME EXCEEDED" << std::endl;
            break;
        }

        // std::cout << "open size " << open.elements.size() << std::endl;
        Node* currentNode = open.pop();

        if (currentNode->cell.x == goalNode->cell.x && currentNode->cell.y == goalNode->cell.y) {
            path.path = make_path(currentNode, startNode, distances, goalNode);

            for (auto node : closed) {
                delete node;
            }
            delete goalNode;

            path.path_length = path.path.size();
            return path;
        }

        std::vector<Node*> kids = expand_node_astar(currentNode, &distances, params);
        // std::cout << "kids expanded to " << kids.size() << " kids " << std::endl;

        for (auto kid : kids) {
            // std::cout << "search kid" << std::endl;
            double cost = g_cost(currentNode, kid, distances, params);

            if (kid->cell.x == goalNode->cell.x && kid->cell.y == goalNode->cell.y) {
                path.path = make_path(kid, startNode, distances, goalNode);

                for (auto node : closed) {
                    delete node;
                }
                delete goalNode;

                path.path_length = path.path.size();
                return path;
            }

            if (is_in_list(kid, closed)) {
                // std::cout << "kid in list, skip" << std::endl;
                continue;
            }
            else {
                if (is_in_list(kid, open.elements)) {
                    // std::cout << "existing kid" << std::endl;
                    Node* existingNode = get_from_list(kid, open.elements);
                    if (cost < existingNode->g_cost) {
                        existingNode->parent = currentNode;
                        existingNode->g_cost = cost;
                        existingNode->h_cost = h_cost(existingNode, goalNode, distances);
                    }
                } 
                else {
                    // std::cout << "new kid" << std::endl;
                    kid->parent = currentNode;
                    kid->g_cost = cost;
                    kid->h_cost = h_cost(kid, goalNode, distances);

                    open.push(kid);
                }
            }
        }
        closed.push_back(currentNode);
    }

    for (auto node : closed) {
        delete node;
    }
    delete goalNode;

    return path;
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

    // Calculate the cost to move from the start node to the current node
    double movement_cost = 1.0; // Assuming each step has a cost of 1

    // Adjust the cost based on obstacle distance
    double distance_cost = 0.0;
    float obstacle_distance = distances(start->cell.x, start->cell.y);
    if (obstacle_distance >= params.minDistanceToObstacle && obstacle_distance <= params.maxDistanceWithCost) {
        distance_cost = pow(params.maxDistanceWithCost - obstacle_distance, params.distanceCostExponent);
    }

    double g_cost = start->g_cost + distance_cost;

    if (dx > 0 && dy > 0) {
        movement_cost = diag_distance;
    }

    g_cost += movement_cost;

    return g_cost;
}

std::vector<Node*> expand_node_astar(Node* node, const ObstacleDistanceGrid* distances, const SearchParams& params) {
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    std::vector<Node*> children;
    for (int n=0; n<8; ++n) {
        int cell_x = node->cell.x + xDeltas[n];
        int cell_y = node->cell.y + yDeltas[n];
        Node* childNode = new Node(cell_x, cell_y);
        childNode->parent = node;

        if(!(distances->isCellInGrid(cell_x, cell_y)))
        {
            // std::cout << "child not in grid: " << "cell x is " << cell_x << " cell y is " << cell_y << " height is " << distances->widthInCells() << " width is " << distances->widthInCells() <<  std::endl;
            continue;
        }
            
        if((*distances)(cell_x, cell_y) <= float(params.minDistanceToObstacle))
        {
            // std::cout << "child in collision" << std::endl;
            delete childNode;
            continue;
        }

        children.push_back(childNode);
    }

    // std::cout << "created " << children.size() << " kids" << std::endl;

    return children;
}

std::vector<pose_xyt_t> make_path(Node* goal_node, Node* start_node, const ObstacleDistanceGrid& distances, Node* given_goal) {
    // Full stack extract path
    std::cout << "Found path!" << std::endl;
    std::cout << "Our goal state: x " << goal_node->cell.x << " y " << goal_node->cell.y << std::endl;

    std::vector<Node*> npath = extract_node_path(goal_node, start_node);
    npath = prune_node_path(npath);
    std::vector<pose_xyt_t> ppath = extract_pose_path(npath, distances);

    for (int i = 0; i < ppath.size(); i++) {
        std::cout << "Pose " << i << " x " << ppath[i].x << " y " << ppath[i].y << std::endl;
    }
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

bool are_nodes_in_line(Node* a, Node* b, Node* c) {
    // Check if the slope between a and b is equal to the slope between b and c
    if (((c->cell.x == b->cell.x) && (a->cell.x == b->cell.x)) || ((c->cell.y == b->cell.y) && (a->cell.y == b->cell.y))) {
        return true;
    }
    else if ((c->cell.x == b->cell.x) || (b->cell.x == a->cell.x)) {
        return false;
    }
    
    return float(float(c->cell.y - b->cell.y) / float(c->cell.x - b->cell.x)) ==
           float(float(b->cell.y - a->cell.y) / float(b->cell.x - a->cell.x));
}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath) {
    // TODO: 3 card monte trimming

    if (nodePath.size() >= 3) {
        std::vector<Node*> pruned_path;
        pruned_path.push_back(nodePath[0]);

        for (size_t i = 1; i < nodePath.size() - 1; ++i) {
            Node* current = nodePath[i];
            Node* previous = nodePath[i - 1];
            Node* next = nodePath[i + 1];

            // Check if the three nodes are in a line
            if (!are_nodes_in_line(previous, current, next)) {
                pruned_path.push_back(current);
            }
        }

        pruned_path.push_back(nodePath.back());
        std::reverse(pruned_path.begin(), pruned_path.end());
        return pruned_path;
    }

    std::reverse(nodePath.begin(), nodePath.end());
    return nodePath;
}

std::vector<pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances) {
    std::vector<pose_xyt_t> path;
    for (auto node : nodes) {
        Node* parent = node->parent;
        Point<double> global_path_cell = grid_position_to_global_position(node->cell, distances);

        pose_xyt_t pose;
        pose.x = float(global_path_cell.x);
        pose.y = float(global_path_cell.y);
        pose.theta = 0.0;

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