/* ******************************
  Copyright 2025 - Ilyes Chaabeni
  Copyright 2021 - Rafael Barreto
 ****************************** */

#include "glwoa_rrtstar_planner/rrt_star.hpp"
#include <chrono>
#include <algorithm> // For std::sort
#include <unordered_map>

namespace glwoa_rrtstar_planner {

// ==================
//    Constructor
// ==================

RRTStar::RRTStar(const std::pair<float, float> &start_point,
                 const std::pair<float, float> &goal_point,
                 costmap_2d::Costmap2D* costmap,
                 double goal_tolerance,
                 double rewiring_radius,
                 double epsilon,
                 unsigned int max_num_nodes,
                 unsigned int min_num_nodes) : 
                                     start_point_(start_point),
                                     goal_point_(goal_point),
                                     costmap_(costmap),
                                     goal_tolerance_(goal_tolerance),
                                     rewiring_radius_(rewiring_radius),
                                     epsilon_(epsilon),
                                     max_num_nodes_(max_num_nodes),
                                     min_num_nodes_(min_num_nodes),
                                     cd_(costmap) {
  nodes_.reserve(max_num_nodes_);
  
  map_bounds = cd_.getMapBounds();
  // Access the lower-left corner coordinates (map)
  double x_min = map_bounds.first.first;
  double y_min = map_bounds.first.second;
  // Access the upper-right corner coordinates (map)
  double x_max = map_bounds.second.first;
  double y_max = map_bounds.second.second;
  
  // **** Setup grid parameters for spatial indexing ***** //

  // Grid cell size based on search radius for efficiency
  grid_cell_size_ = rewiring_radius_ * 2.0;
  
  // Set grid bounds
  grid_min_x_ = x_min;
  grid_min_y_ = y_min;
  grid_width_ = std::ceil((x_max - x_min) / grid_cell_size_);
  grid_height_ = std::ceil((y_max - y_min) / grid_cell_size_);
  // ROS_INFO("Spatial grid size: %d x %d, Cell size (m): %.2f", grid_width_, grid_height_, grid_cell_size_);
  
  // Reserve space for the grid
  spatial_grid_.resize(grid_width_ * grid_height_);
  
  /*
  * Pre-allocate `near_node_indices` to avoid repeated allocations.
  *
  * Rationale:
  * - Assume nodes are uniformly distributed in the space between the start and goal.
  * - Use the maximum number of nodes to compute the maximum required size for reservation.
  * - This provides a safe upper bound on memory use, ensuring performance without over-allocating.
  *
  * Notes:
  * - The actual number of nodes (density) may be much lower due to:
  *     -Space being partially occupied,
  *     -Portions of the space being outside the start-goal corridor that can be used.
  * - A more accurate estimate could consider occupancy rate or spatial constraints,
  *   but this would result in a lower reservation size and add complexity.
  */
  unsigned int reserved_near_nodes_size;
  // get the number of grid cells between start and goal
  int g_x = std::ceil((std::abs(goal_point_.first - start_point_.first)) / grid_cell_size_);
  int g_y = std::ceil((std::abs(goal_point_.second - start_point_.second)) / grid_cell_size_);
  // Estimate the max number of near nodes in 4 cells
  reserved_near_nodes_size = std::ceil(4 * max_num_nodes_ / (g_x * g_y));
  // Reserve space for near_node_indices_
  near_node_indices_.reserve(std::max(reserved_near_nodes_size, max_num_nodes_));
  
  // Set generator range (search space)
  random_double_.setRangeFirst(x_min, x_max);
  random_double_.setRangeSecond(y_min, y_max);
}

// ==========================================
//  Finding an Initial Path (Main Algorithm)
// ==========================================

bool RRTStar::initialPath(std::list<std::pair<float, float>> &path) {
  goal_reached_ = false;
  // ensure goal point is not colliding
  if (cd_.isThisPointCollides(goal_point_.first, goal_point_.second)) {
    ROS_ERROR("Goal point chosen is NOT in the FREE SPACE! Choose other goal!");
    return false;
  }
  // ensure start point is not colliding
  if (cd_.isThisPointCollides(start_point_.first, start_point_.second)) {
    ROS_ERROR("The start point is not in free space. The robot seems to be in the inflation of an obstacle or has wrong position!");
    return false;
  }

  // Start Node
  createNewNode(start_point_.first, start_point_.second, -1, {});

  std::pair<float, float> p_rand, p_new;
  Node node_nearest;
  bool found_next;
  // float tries_avg = 0.0; // number of tries to find a new node that doesn't collide
  auto start_time = std::chrono::steady_clock::now();

  // // Add timing accumulators (for evaluating performance)
  // time_nearest_node_ = 0.0; // time spent in getNearestNodeId
  // time_near_nodes_ = 0.0; // time spent in getNearNodes
  // time_collision_check_ = 0.0; // time spent in collision checks

  // **** main loop **** //
  while (nodes_.size() < max_num_nodes_ && !goal_reached_) {
    found_next = false;

    /* continue generating a random point until it is possible 
    without any collision */
    while (!found_next) {
      p_rand = sampleFree();  // random point in the free space

      // --- get the nearest node index ---
      // auto t1 = std::chrono::high_resolution_clock::now();
      int nearest_id = getNearestNodeId(p_rand);
      node_nearest = nodes_[nearest_id];
      // auto t2 = std::chrono::high_resolution_clock::now();
      // time_nearest_node_ += std::chrono::duration<double>(t2 - t1).count();

      // --- steer ---
      p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);

      // --- collision checks ---
      // auto tcol1 = std::chrono::high_resolution_clock::now();
      bool obstacle_between = cd_.isThereObstacleBetween(node_nearest, p_new);
      bool point_collides = cd_.isThisPointCollides(p_new.first, p_new.second);
      // auto tcol2 = std::chrono::high_resolution_clock::now();
      // time_collision_check_ += std::chrono::duration<double>(tcol2 - tcol1).count();

      // if no obstacle between the nearest node and the new point (points included)
      // exit the loop and create the new node
      if (!obstacle_between && !point_collides) {
        found_next = true;

        // --- get the near nodes (rewiring radius) ---
        // auto t3 = std::chrono::high_resolution_clock::now();
        near_node_indices_.clear();
        getNearNodes(p_new, rewiring_radius_, near_node_indices_);
        // auto t4 = std::chrono::high_resolution_clock::now();
        // time_near_nodes_ += std::chrono::duration<double>(t4 - t3).count();
        
        // add the new node to the tree
        createNewNode(p_new.first, p_new.second, nearest_id, near_node_indices_);
      }
      // tries_avg += 1.0; // point collides: we should try again
    }
    
    // after p_new is generated, check if it is within goal's vicinity
    goal_reached_ = isGoalReached(p_new);
  }

  // if goal reached: initial path found
  if(goal_reached_) {
    goal_node_ = nodes_.back();
    ROS_INFO("Initial Path found!");
    ROS_INFO("Number of nodes: %ld", nodes_.size());
    computeFinalPath(path);
    auto end_time = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration<double, std::milli>(end_time - start_time);
    ROS_INFO("---> Time taken to find initial path: %.2f ms", diff.count());
    // // Total time spent in getNearestNodeId, getNearNodes and collision checks
    // ROS_INFO("---> Total time taken by getNearestNodeId: %f seconds", time_nearest_node_);
    // ROS_INFO("---> Total time taken by getNearNodes: %f seconds", time_near_nodes_);
    // ROS_INFO("---> Total time taken by both: %f seconds", time_nearest_node_ + time_near_nodes_);
    // ROS_INFO("---> Total time taken by collision checks: %f seconds", time_collision_check_);
    return true;
  }
  // max number of nodes reached
  else{
    max_nodes_reached_ = true;
    ROS_ERROR("Max number of nodes reached! Did not find a path!");
    // ROS_INFO("Tries average: %.2f", tries_avg/nodes_.size());
    return false;
  }
}

// ==================
//    Refine Path
// ==================
// not used, not optimized

bool RRTStar::refinePath(std::list<std::pair<float, float>> &path) {

  if (cd_.isThisPointCollides(goal_point_.first, goal_point_.second)) {
    ROS_ERROR("Goal point chosen is NOT in the FREE SPACE! Choose other goal!");
    return false;
  }

  std::pair<float, float> p_rand;
  std::pair<float, float> p_new;

  Node node_nearest;

  bool found_next;
  // float tries_avg=0.0;
  auto start_time = std::chrono::high_resolution_clock::now();
  if (nodes_.size() > min_num_nodes_) {
    ROS_INFO("Minimum number of nodes exceeded. Path will not be refined.");
    return false;
  }
  // main loop
  while (nodes_.size() < min_num_nodes_) {
    found_next = false;

    while (!found_next) {
      /* continue generating a random point until it is possible 
      without any collision */
      p_rand = sampleFree();  // random point in the free space
      node_nearest = nodes_[getNearestNodeId(p_rand)];  // nearest node of the random point
      p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);  // new point and node candidate
      if (!cd_.isThereObstacleBetween(node_nearest, p_new) && !cd_.isThisPointCollides(p_new.first, p_new.second)) {
        found_next = true;
        createNewNode(p_new.first, p_new.second, node_nearest.node_id, {});
      }
      // tries_avg+=1.0;
    }
    // after p_new is generated, check if it is within goal's vicinity
    // goal_reached_=isGoalReached(p_new);
  }
  ROS_INFO("RRT* Path Refined.");
  // ROS_INFO("Refined Path Cost: %.4f", goal_node_.cost);
  computeFinalPath(path);
  return true;
  }



// ==============================
//  sampleFree (2D random point)
// ==============================

std::pair<float, float> RRTStar::sampleFree() {
  // sample a random point in the search space
  std::pair<float, float> random_point;
  random_point.first = random_double_.generateFirst();
  random_point.second = random_double_.generateSecond();
  return random_point;
}

// ========================
//     getNearestNodeId
// ========================
// using a more efficient approach with spatial grid search
// Only checks the necessary grid cells (at most 4) rather than all 9 surrounding cells

int RRTStar::getNearestNodeId(const std::pair<float, float> &point) {
  if (nodes_.size() <= 100) {  // Use brute force for very small trees
    return getNearestNodeBruteForce(point);
  }
  
  // Calculate exact floating-point cell coordinates
  float exact_cell_x = (point.first - grid_min_x_) / grid_cell_size_;
  float exact_cell_y = (point.second - grid_min_y_) / grid_cell_size_;
  
  // Get integer cell coordinates
  int base_cell_x = static_cast<int>(exact_cell_x);
  int base_cell_y = static_cast<int>(exact_cell_y);
  
  // Fractional parts determine which adjacent cells to check
  float frac_x = exact_cell_x - base_cell_x;
  float frac_y = exact_cell_y - base_cell_y;
  
  // Determine which cells to check based on position within the cell
  // If we're in the right half of the cell, check current and right cell
  // If we're in the left half, check current and left cell
  int start_x = frac_x > 0.5f ? base_cell_x : base_cell_x - 1; // starting index on x
  int end_x = start_x + 1; // end index on x
  
  // Similarly for the y-direction
  int start_y = frac_y > 0.5f ? base_cell_y : base_cell_y - 1; // starting index on y
  int end_y = start_y + 1; // end index on y
  
  // Ensure we're within grid bounds
  start_x = std::max(0, start_x);
  end_x = std::min(grid_width_ - 1, end_x);
  start_y = std::max(0, start_y);
  end_y = std::min(grid_height_ - 1, end_y);
  
  float min_dist = std::numeric_limits<float>::max(); // minimum distance found so far
  int nearest_id = 0; // nearest node id found
  bool found_in_cell = false; // flag to check if we found a node in the immediate vicinity
  
  // Check only the necessary cells (at most 4, often fewer)
  for (int y = start_y; y <= end_y; y++) {
    for (int x = start_x; x <= end_x; x++) {
      int cell_idx = y * grid_width_ + x;
      
      // Check each node in this cell
      for (int node_id : spatial_grid_[cell_idx]) {
        const Node& node = nodes_[node_id];
        float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);
        if (dist < min_dist) {
          min_dist = dist;
          nearest_id = node_id;
          found_in_cell = true;
        }
      }
    }
  }
  
  // If we didn't find anything in the immediate vicinity, expand search using brute force
  // This is a fallback to ensure we always return a valid node
  if (!found_in_cell) {
    return getNearestNodeBruteForce(point);
  }
  
  return nearest_id;
}

// Fallback brute force method
int RRTStar::getNearestNodeBruteForce(const std::pair<float, float> &point) {
  float min_dist = std::numeric_limits<float>::max();
  int nearest_id = 0;
  
  for (int i = 0; i < nodes_.size(); ++i) {
    float dist = euclideanDistance2D(nodes_[i].x, nodes_[i].y, point.first, point.second);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_id = i;
    }
  }
  return nearest_id;
}

// ===================
//    getNearNodes
// ===================
// Get nodes within a certain radius - using spacial grid search

void RRTStar::getNearNodes(const std::pair<float, float> &point, float radius, std::vector<int> &result) {
  result.clear();
  
  // Calculate exact floating-point cell coordinates
  float exact_cell_x = (point.first - grid_min_x_) / grid_cell_size_;
  float exact_cell_y = (point.second - grid_min_y_) / grid_cell_size_;
  
  // Get integer cell coordinates
  int base_cell_x = static_cast<int>(exact_cell_x);
  int base_cell_y = static_cast<int>(exact_cell_y);
  
  // Fractional parts determine which adjacent cells to check
  float frac_x = exact_cell_x - base_cell_x;
  float frac_y = exact_cell_y - base_cell_y;
  
  // Determine which cells to check based on position within the cell
  // If we're in the right half of the cell, check current and right cell
  // If we're in the left half, check current and left cell
  int start_x = frac_x > 0.5f ? base_cell_x : base_cell_x - 1;
  int end_x = start_x + 1;
  
  // Similarly for y-direction
  int start_y = frac_y > 0.5f ? base_cell_y : base_cell_y - 1;
  int end_y = start_y + 1;
  
  // Ensure we're within grid bounds
  start_x = std::max(0, start_x);
  end_x = std::min(grid_width_ - 1, end_x);
  start_y = std::max(0, start_y);
  end_y = std::min(grid_height_ - 1, end_y);
  // ROS_INFO("Number of cells to check: (%d, %d)", end_x - start_x, end_y - start_y);
  
  // Check only the necessary cells (at most 4, often fewer)
  for (int y = start_y; y <= end_y; y++) {
    for (int x = start_x; x <= end_x; x++) {
      int cell_idx = y * grid_width_ + x;
      
      // Check each node in this cell
      for (int node_id : spatial_grid_[cell_idx]) {
        const Node& node = nodes_[node_id];
        float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);
        if (dist < radius) {
          result.push_back(node_id);
        }
      }
    }
  }
}

// =============================
//    createNewNode (improved) 
// =============================
// Create a new node, choose its parent, and rewire the tree.
// Also updates the spatial grid with the new node.

void RRTStar::createNewNode(float x, float y, int nearest_node_id, const std::vector<int>& near_indices) {
  // new node placed using steer
  Node new_node(x, y, node_count_, nearest_node_id);
  nodes_.emplace_back(new_node);
  
  // Update spatial grid with the new node
  updateSpatialGrid(new_node);

  if (nearest_node_id != -1) {
    // Optimize using near nodes found using spatial grid search
    if (!near_indices.empty()) {
      // Use the provided near nodes indices
      chooseParent(nearest_node_id, near_indices);
      rewire(near_indices);
    } else {
      // Fall back to original methods if near_indices wasn't provided
      ROS_WARN("No near nodes found, falling back to original parent selection and rewiring.");
      chooseParentBruteForce(nearest_node_id);
      rewireBruteForce();
    }
  }
  node_count_++;
}

// ========================
//    updateSpatialGrid
// ========================
// Update the grid vector where each element has as index the grid's index 
// When adding a node, update the spatial grid (required) using this function.

void RRTStar::updateSpatialGrid(const Node& node) {
  int cell_x = static_cast<int>((node.x - grid_min_x_) / grid_cell_size_);
  int cell_y = static_cast<int>((node.y - grid_min_y_) / grid_cell_size_);
  
  // Ensure we're within grid bounds
  if (cell_x >= 0 && cell_x < grid_width_ && cell_y >= 0 && cell_y < grid_height_) {
    int cell_idx = cell_y * grid_width_ + cell_x;
    spatial_grid_[cell_idx].push_back(node.node_id);
  }
}

// ============================
//   chooseParent (improved) 
// ============================

void RRTStar::chooseParent(int nearest_node_id, const std::vector<int>& near_indices) {
  float cost_new_node;
  float cost_other_parent;
  float nodes_dist;
  // parent node initialized to nearest node
  Node parent_node = nodes_[nearest_node_id];
  Node &new_node = nodes_.back();

  // Use the provided near nodes (more efficient) rather than checking all nodes
  for (int idx : near_indices) {
    const auto& node = nodes_[idx];
    if (node.node_id == new_node.node_id) continue;
    
    nodes_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);
    cost_new_node = parent_node.cost + euclideanDistance2D(parent_node.x, parent_node.y, new_node.x, new_node.y);
    cost_other_parent = node.cost + nodes_dist;
    
    if (cost_other_parent < cost_new_node) {
      if (!cd_.isThereObstacleBetween(node, new_node)) {
        parent_node = node;
      }
    }
  }
  // Update new_node cost and its new parent
  new_node.cost = parent_node.cost + euclideanDistance2D(parent_node.x, parent_node.y, new_node.x, new_node.y);
  new_node.parent_id = parent_node.node_id;
}

// ===============================
//   chooseParentBruteForce (old)
// ===============================

void RRTStar::chooseParentBruteForce(int nearest_node_id) {
  float cost_new_node;
  float cost_other_parent;
  float nodes_dist;
  // parent node initialized to nearest node
  Node parent_node = nodes_[nearest_node_id];

  Node &new_node = nodes_.back();

  for (const auto &node : nodes_) {
    if (node.node_id == new_node.node_id) continue;
    // distance between node and new_node
    nodes_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);

    if (nodes_dist < rewiring_radius_) {
      // current cost of new_node
      cost_new_node = parent_node.cost + euclideanDistance2D(parent_node.x, parent_node.y, new_node.x, new_node.y);

      // cost if the parent is node
      cost_other_parent = node.cost + nodes_dist;

      if (cost_other_parent < cost_new_node) {
        // node is a better parent choice 
        if (!cd_.isThereObstacleBetween(node, new_node)) {
          parent_node = node;
        }
      }
    }
  }
  // Update new_node cost and its new parent
  new_node.cost = parent_node.cost + euclideanDistance2D(parent_node.x, parent_node.y, new_node.x, new_node.y);
  new_node.parent_id = parent_node.node_id;
}

// ======================
//   rewire (improved)
// ======================

void RRTStar::rewire(const std::vector<int>& near_indices) {
  float cost_node;
  Node new_node = nodes_.back();

  // Use the provided near nodes (more efficient)
  for (int idx : near_indices) {
    auto& node = nodes_[idx];
    if (node.node_id == new_node.parent_id) continue;
    
    cost_node = new_node.cost + euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);
    
    if (cost_node < node.cost && !cd_.isThereObstacleBetween(node, new_node)) {
      node.parent_id = new_node.node_id;
      node.cost = cost_node;
    }
  }
}

// =========================
//   rewireBruteForce (old)
// =========================

void RRTStar::rewireBruteForce() {
  float nodes_dist;
  float cost_node;

  Node new_node = nodes_.back();

  for (auto &node : nodes_) {
    // distance between node and new_node
    nodes_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);

    // check if node is already the parent and if node is near for optimization
    if (node != nodes_[new_node.parent_id] && nodes_dist < rewiring_radius_) {
      // cost if the parent of node is new_node
      cost_node = new_node.cost + euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);

      if (cost_node < node.cost && !cd_.isThereObstacleBetween(node, new_node)) {
        // update the new parent of node and its new cost
        node.parent_id = new_node.node_id;
        node.cost = cost_node;
      }
    }
  }
}

// ============
//    steer
// ============

std::pair<float, float> RRTStar::steer(float x1, float y1, float x2, float y2) {
  std::pair<float, float> p_new;
  float dist = euclideanDistance2D(x1, y1, x2, y2);
  if (dist < epsilon_) {
    p_new.first = x1;
    p_new.second = y1;
    return p_new;
  } else {
    float theta = atan2(y2 - y1, x2 - x1);
    p_new.first = x1 + epsilon_ * cos(theta);
    p_new.second = y1 + epsilon_ * sin(theta);
    return p_new;
  }
}

// ======================
//   computeFinalPath
// ======================
// using backtracking from the goal start (moving through parent nodes)
// find the path from the start node to the goal node

void RRTStar::computeFinalPath(std::list<std::pair<float, float>> &path) {
  path.clear(); // clean previous path
  path_cost_ = 0.0; // reset previous cost
  // Compute the path from the goal to the start
  Node current_node = goal_node_;
  // double path_cost=0;
  // Final Path
  std::pair<float, float> point;
  std::pair<float, float> prev_point=std::make_pair(goal_node_.x, goal_node_.y);

  do {
    point.first = current_node.x;
    point.second = current_node.y;
    path.push_front(point);
    path_cost_+=euclideanDistance2D(point.first, point.second, prev_point.first, prev_point.second);

    // update the current node
    prev_point = point;
    current_node = nodes_[current_node.parent_id];
  } while (current_node.parent_id != -1);
  
  point.first = current_node.x;
  point.second = current_node.y; 
  path.push_front(point);
  path_cost_+=euclideanDistance2D(point.first, point.second, prev_point.first, prev_point.second);  
  ROS_INFO("Path cost: %f", path_cost_);
}

}