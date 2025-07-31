/* ******************************
  Copyright 2025 - Ilyes Chaabeni
 ****************************** */

#include "glwoa_rrtstar_planner/woa_agent.hpp"
#include <costmap_2d/costmap_2d.h>
#include <iostream>

namespace glwoa_rrtstar_planner {

// ==================
//    Constructor
// ==================

PathAgent::PathAgent(std::list<std::pair<float, float>> &path,
                     const float sampling_radius,
                     uint16_t id,
                     costmap_2d::Costmap2D* costmap,
                     float spiral_shape): id_(id),
                                          path_(path),
                                          sampling_radius_(sampling_radius),
                                          collision_(costmap),
                                          b(spiral_shape),
                                          costmap_(costmap){

    std::pair<std::pair<double, double>, std::pair<double, double>> map_bounds;
    map_bounds = collision_.getMapBounds();
    // Access the lower-left corner coordinates
    x_min_ = map_bounds.first.first;
    y_min_ = map_bounds.first.second;
    // Access the upper-right corner coordinates
    x_max_ = map_bounds.second.first;
    y_max_ = map_bounds.second.second;
    // get the start and goal points
    start_point_= path_.front();
    goal_point_ = path_.back();

    /* **** Initialize each agent's path **** */
    if (id_==0) initial_path_=path_; // the first agent gets RRT* initial path
    else{
      path_ = randomInitialPath(path); // use the procedure described in the article
      // ROS_INFO("Random path generated for agent %d", id_);
      initial_path_=path_;
    }

    // Agent vector (X) excludes start and goal points (they are fixed points)
    // X = [x1 y1 x2 y2 x3 y3 ... xn yn]
    vec_size = static_cast<uint16_t>(2*path_.size()-4);
    // Initialize X
    X.set_size(vec_size);
    // iterator to traverse the list
    auto it = std::next(path_.begin()); // skip the start point
    // Fill the vector X with the path points
    for (int i = 0; i < vec_size; i += 2) {
        X.at(i) = static_cast<float>(it->first);
        X.at(i+1) = static_cast<float>(it->second);
        ++it;  // Move to the next element in the list
    }
    // Initialize D and D2
    D = arma::zeros<arma::vec>(vec_size); 
    D2 = arma::zeros<arma::vec>(vec_size); 
}

// =========================
//    Random Initial Path
// =========================

std::list<std::pair<float, float>> PathAgent::randomInitialPath(const std::list<std::pair<float,float>> &path) {
  std::list<std::pair<float, float>> rand_path;
  std::pair<float, float> p_rand, p_new;
  bool found_next=false;
  int num_travels_= 0;
  auto it = path.begin();  // Iterator to traverse the list
  rand_path.push_back(*it); // Use push_back instead of push_front for consistent insertion
  ++it;
  int i=2;
  // starting from the second node

  // main loop
  while (std::next(it) != path.end()) {
    found_next = false; // Reset found_next before the inner loop
    int iterations = 0; // keep track of iterations to avoid infinite loops
    while (!found_next) {
      // biased sampling with current node as center of the circle   
      p_rand = biasedSampling(*it);
      auto prev_it = std::prev(it);   
      // Ensure *prev_it and p_rand are of type std::pair<double, double>
      auto prev_point = std::make_pair(static_cast<double>(prev_it->first), static_cast<double>(prev_it->second));
      auto rand_point = std::make_pair(static_cast<double>(p_rand.first), static_cast<double>(p_rand.second));
      // check if the newly generated point doesn't collide with the previous point
      if (!collision_.isThereObstacleBetween(prev_point, rand_point)) {
        found_next = true;
        rand_path.push_back(rand_point);
      }
      iterations++;
      if (iterations > 100) {
        // If we can't find a point after 100 tries, use the original path points
        // replace the last point in rand_path with the previous point from the original path
        // ROS_WARN("Could not find a valid point n°%d for agent %d", i, id_);
        if (!rand_path.empty()) {
          rand_path.pop_back(); // Remove the last attempted point
        }
        rand_path.push_back(*std::prev(it)); // Add the previous point from original path
        rand_path.push_back(*it);           // Add the current point from original path
        found_next = true;
      }
    }
    i++; // Increment the index for the next point
    ++it; // Move to the next element in the list
  }
  // end of main loop
  rand_path.push_back(path.back()); // Add the last point
  
  // Final verification to ensure path lengths match
  if (rand_path.size() != path.size()) {
    ROS_ERROR("Path size mismatch during creation. Reverting to original path.");
    return path; // Return the original path if sizes don't match
  }
  return rand_path;
}

// ======================
//    Biased Sampling
// ======================

std::pair<float, float> PathAgent::biasedSampling(const std::pair<float, float> &center) {
  std::pair<float, float> new_rand_point;
  double min_x = center.first - sampling_radius_;
  double max_x = center.first + sampling_radius_;
  double min_y = center.second - sampling_radius_;
  double max_y = center.second + sampling_radius_;
  // use the random device to generate a point within the specified bounds
  random_device_.setRangeFirst(min_x, max_x);
  new_rand_point.first = random_device_.generateFirst();
  random_device_.setRangeSecond(min_y, max_y);
  new_rand_point.second = random_device_.generateSecond();
  
  return new_rand_point;
}

// ======================
//    Circular update
// ======================

void PathAgent::circularUpdate(const arma::vec& search_agent) {
  // Update the distance according to Eq. (2.1) and (2.7) in the original WOA article
  // search_agent is either Xbest or Xrand
  D=arma::abs(C*search_agent-X);
  // Update X according to Eq. (2.8)
  X=search_agent-A*D;

  // check for invalid values
  if (X.has_nan()) {
      ROS_ERROR("X contains NaN values!");
  }
  if (X.has_inf()) {
      ROS_ERROR("X contains Inf values!");
  }
  // update X despite colliding or being out of bounds
  // clampToBounds will be used later
}

// =====================
//     Spiral update
// =====================

void PathAgent::spiralUpdate(const arma::vec& search_agent) {
  // Update the distance D'
  D2=arma::abs(search_agent-X);
  // Update the agent vector according to Eq. (2.5) in the original WOA article
  X=search_agent+(std::exp(b*l)*std::cos(2*M_PI*l))*D2;

  // check for invalid values
  if (X.has_nan()) {
      ROS_ERROR("X contains NaN values!");
  }
  if (X.has_inf()) {
      ROS_ERROR("X contains Inf values!");
  }
  // update X despite colliding or being out of bounds
  // clampToBounds will be used later
}

// =====================
//     ClampToBounds
// =====================

bool PathAgent::clampToBounds(){
  bool is_out_of_bounds = false;
  // clamp all points in the agent's vector to the map bounds
  for (int k=0; k<vec_size; k+=2){
    // check X coordinates
    if (X.at(k) < x_min_ || X.at(k) > x_max_){
      X.at(k)=std::clamp(X.at(k), x_min_, x_max_);
      is_out_of_bounds = true;
    }
    // check Y coordinates
    if (X.at(k+1) < y_min_ || X.at(k+1) > y_max_) {
      X.at(k+1)= std::clamp(X.at(k+1), y_min_, y_max_);
      is_out_of_bounds = true;
    }
  }
  // Return whether the path was initially out of bounds
  return is_out_of_bounds; // output used for debugging purposes
}

/*
 fitness function: path length
*/
double PathAgent::fitness() {
    double cost=0.0;
    // euclideanDistance2D(x1,y1,x2,y2)
    cost+=euclideanDistance2D(start_point_.first, start_point_.second, X.at(0), X.at(1));
    if (vec_size>2){
      for(int i=0; i<vec_size-2; i+=2){
        cost+=euclideanDistance2D(X.at(i), X.at(i+1), X.at(i+2), X.at(i+3));
      }
    }
    cost+=euclideanDistance2D(goal_point_.first, goal_point_.second, X.at(vec_size-2), X.at(vec_size-1));
    // ROS_INFO("Adding distance between the two points: goal (%.4f, %.4f) and (%.4f, %.4f)", goal_point_.first, goal_point_.second, X.at(vec_size-2), X.at(vec_size-1));
    // ROS_INFO("Cost after adding the goal point: %.4f", cost);
    if (cost<0) {
      ROS_WARN("Cost is negative");
    }
    return cost;
}

// i= 0  1  2  3  4  5  6  7
//    x1 y1 x2 y2 x3 y3 x4 y4
// iter 1: dist(P1, P2) 
// iter 2: dist(P2, P3)
// iter n-4: dist(Pn-4, Pn-2)


/*
Index of the agent (id)
*/ 
uint16_t PathAgent::getID(){
  return id_;
}


/*
doesPathCollides
*/
bool PathAgent::doesPathCollide(){
  // if 1st point collides or obstacle between 1st point and start_point_
  if (collision_.isThisPointCollides(X.at(0), X.at(1)) || collision_.isThereObstacleBetween(start_point_, std::make_pair(X.at(0), X.at(1)))){
    return true;
    // ROS_WARN("Start point collides with next point");
  }
  // if last point collides or obstacle between last point and goal point 
  if (collision_.isThisPointCollides(X.at(vec_size-2), X.at(vec_size-1)) || collision_.isThereObstacleBetween(goal_point_, std::make_pair(X.at(vec_size-2), X.at(vec_size-1)))){
    return true;
    // ROS_WARN("Goal point collides with previous point");
  }

  // collision check for each new point in X and between its preceeding point 
  int k=2;
  
  while (k<vec_size){
    if (collision_.isThisPointCollides(X.at(k), X.at(k+1))){
      return true;
      // ROS_WARN("There is collision in X (spiral)");
    }
    if (collision_.isThereObstacleBetween(std::make_pair(X.at(k-2), X.at(k-1)), std::make_pair(X.at(k), X.at(k+1)))){
      return true;
      // ROS_WARN("There is collision in X (spiral)");
    }
    k+=2;
  }
  return false;
}



}