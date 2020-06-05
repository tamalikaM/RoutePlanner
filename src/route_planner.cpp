#include "route_planner.h"
#include <algorithm>

using std::vector;
using std::sort;
using std::reverse;
using std::begin;
using std::end;
using std::cout;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Using m_Model.FindClosestNode() to find the closest nodes to the starting and ending coordinates.
    // Store the nodes in RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}

// - Using distance to the end_node for calculating h value. 
// - Euclidean distance is used as a heuiristic function
// - Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return end_node->distance(*node);
}


// Expand the current node by adding all unvisited neighbors to the open list.
// - Using FindNeighbors() method of current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
    for(RouteModel::Node *node : current_node->neighbors ){
      if(node -> visited == false) {  
      	node->parent = current_node;
        node->g_value = current_node->g_value + node->distance(*current_node);
        node->h_value = CalculateHValue(node);
        open_list.push_back(node);
        node->visited =true;
      }
    }
}

bool Compare(RouteModel::Node *nodeA, RouteModel::Node *nodeB) {
    double comparisonfactor1 = nodeA->h_value + nodeA->g_value;
    double comparisonfactor2 = nodeB->h_value + nodeB->g_value;
    return comparisonfactor1 > comparisonfactor2;
}

void CellSort(vector<RouteModel::Node*> *open) {
    sort(open->begin(), open->end(), Compare);
}


// Sort the open list and return the next node.
// - Sort the open_list based on the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
  
  	CellSort(&open_list);
    RouteModel::Node *nextNode = open_list.back();
    open_list.pop_back();
    return nextNode;
}


// Return the final path found from our A* search.
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *parent_node;
    while(current_node != start_node){
        parent_node = current_node->parent;
        distance += parent_node->distance(*current_node);
        path_found.push_back(*current_node);
        current_node = parent_node;
    }
	path_found.push_back(*current_node);
    reverse(begin(path_found),end(path_found));

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm here.
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
   
    start_node-> visited = true;
  	open_list.push_back(start_node);
    while(!open_list.empty()) {
        current_node = NextNode();
        if(current_node == end_node)
            break;
        AddNeighbors(current_node);    
    }
    vector<RouteModel::Node> path_found = ConstructFinalPath(current_node);
    m_Model.path = path_found;

}