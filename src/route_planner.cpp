#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &(model.FindClosestNode(start_x, start_y));
  	end_node = &(model.FindClosestNode(end_x, end_y));
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float dist = node->distance(*end_node);
    return dist;
}


// FindNeighbors() populates current_node.neighbors vector with all the neighbors.
// For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
  
  for(auto neighbor: current_node->neighbors){
  	neighbor->parent = current_node;
    neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
  	neighbor->h_value = CalculateHValue(neighbor);    
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }

}


// Sort the open_list according to the sum of the h value and g value.

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(), open_list.end(), Compare);
  RouteModel::Node *last = open_list.back();
  open_list.pop_back();
  return last;
}


// For each node in the chain, add the distance from the node to its parent to the distance variable.
// The returned vector should be in the correct order: the start node should be the first element
// of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    RouteModel::Node *curr = current_node;
	while(curr != start_node){
      distance += curr->distance(*(curr->parent));
      path_found.push_back(*curr);
      curr = curr->parent;
    }
  	path_found.push_back(*curr);
  
  	std::reverse(path_found.begin(), path_found.end());
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
void RoutePlanner::AStarSearch() {
   RouteModel::Node *current_node = start_node;
	
  	current_node->visited = true;
  	open_list.push_back(current_node);  
     
  	while(open_list.size() > 0){
        
      if(current_node == end_node){
        m_Model.path = ConstructFinalPath(current_node); 
        break;
      }
      
      AddNeighbors(current_node);
      current_node = NextNode();
      current_node->visited = true;
    } 
  
}
