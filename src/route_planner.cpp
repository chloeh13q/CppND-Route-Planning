#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find closest nodes to starting and ending coordinates.
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}



// Calculate h value (cost to travel from the starting node to the current node).
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return end_node->distance(*node);
}


// Expand the current node by adding all unvisited neighbors to the open list after
// initiating their respective parent, g_value, h_value, and visited status.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node* i : current_node->neighbors) {
        i->parent = current_node;
        i->g_value = current_node->g_value + current_node->distance(*i);
        i->h_value = CalculateHValue(i);
        i->visited = true;
        open_list.push_back(i);
    }
}


// Sort the open list and return the next node with the smallest f value.
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(),
              [](const RouteModel::Node* node1, const RouteModel::Node* node2) -> bool
    {
        return node1->h_value + node1->g_value > node2->h_value + node2->g_value;
    });
    RouteModel::Node* output_node = open_list[open_list.size() - 1];
    open_list.pop_back();
    return output_node;
}


// Return the final path found from A* search by iteratively finding each node's
// parent, starting from the end node until the starting node is reached.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.emplace_back(*current_node);
    while (current_node->parent) {
        path_found.emplace_back(*(current_node->parent));
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
    return path_found;
}


// Perform A* Search algorithm. After adding the starting node to the open list,
// iteratively use the AddNeighbors method to add all of the neighbors of the
// current node and then use the NextNode method to sort and find the next node.
// If open list becomes empty or the end node is reached, the ConstructFinalPath
// method is called to store the complete path in the path attribute. Here assume
// that a path can always be found between two input points.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    start_node->visited = true;
    open_list.emplace_back(current_node);
    while (!open_list.empty() && current_node->distance(*end_node) >= 0.001) {
        RoutePlanner::AddNeighbors(current_node);
        current_node = RoutePlanner::NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);
}
