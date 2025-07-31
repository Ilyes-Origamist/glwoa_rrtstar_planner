/* ******************************
  Copyright 2025 - Ilyes Chaabeni
  Copyright 2021 - Rafael Barreto
 ****************************** */

#include "glwoa_rrtstar_planner/collision_detector.hpp"
#include <ros/ros.h>

namespace glwoa_rrtstar_planner {

CollisionDetector::CollisionDetector(costmap_2d::Costmap2D* costmap) : costmap_(costmap) {
  if (costmap_ != nullptr) {
    resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
  }
  else ROS_ERROR("NULL pointer for costmap");
}

bool CollisionDetector::isThisPointCollides(float wx, float wy) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    // no collision
    return false;
  }

  unsigned int mx, my;
  // worldToMap(wx, wy, mx, my);
  bool check = costmap_->worldToMap(wx, wy, mx, my);
  // boundary check
  if ((mx < 0) || (my < 0) || (mx > costmap_->getSizeInCellsX()) || (my > costmap_->getSizeInCellsY())){
    // ROS_WARN("Point (%.4f, %.4f) is out of map", wx, wy);
    // ROS_WARN("Conversion state: %d", check);
    return true;
  }
  // getCost returns unsigned char
  unsigned int cost = static_cast<int>(costmap_->getCost(mx, my));
  // ROS_INFO("Point (%.4f, %.4f) has cost: %u", wx, wy, cost);
  // cost > 127 means obstacle
  // cost = 0 means totally free space
  if (cost > 0){
    // ROS_INFO("Point at obstacle: (%.4f, %.4f)", wx, wy);
    return true;
  }
  else{
    // ROS_WARN("Chek if this point (%.4f, %.4f) is not in obstacle", wx, wy);
    return false;
  }
}
bool CollisionDetector::isThereObstacleBetween(const Node &node, const std::pair<double, double> &point) {
  // ...

  float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);

  if (dist < resolution_) {
    return (isThisPointCollides(point.first, point.second)) ? true : false;
  } 
  else {
    // check if last point collides
    if (isThisPointCollides(point.first, point.second)){
      return true;
    }
    // compute step number
    int steps_number = static_cast<int>(floor(dist/(resolution_)));
    // ROS_INFO("Steps number: %d", steps_number);

    float theta = atan2(point.second - node.y, point.first - node.x);
    // ROS_INFO("Theta: %.4f", theta);

    std::pair<float, float> p_n;
    for (int n = 1; n < steps_number; n++) {
      p_n.first = node.x + n*resolution_*cos(theta);
      p_n.second = node.y + n*resolution_*sin(theta);
      // ROS_INFO("Checking point (%.4f, %.4f)", p_n.first, p_n.second);
      if (isThisPointCollides(p_n.first, p_n.second)) {
        // ROS_INFO("Collision detected at point (%.4f, %.4f)", p_n.first, p_n.second);
        return true;
      }
      // ROS_INFO("No collision detected");
    }
    return false;
  }
}

// Collision test between two points
bool CollisionDetector::isThereObstacleBetween(const std::pair<double, double> &point1, const std::pair<double, double> &point2) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    // there is NO obstacles
    return false;
  }

  // Calculate the distance between the two points
  float dist = euclideanDistance2D(point1.first, point1.second, point2.first, point2.second);

  // If the distance is less than the resolution, check the endpoint directly
  if (dist < resolution_) {
    return isThisPointCollides(point2.first, point2.second);
  } else {
    // Check if the endpoint collides
    if (isThisPointCollides(point2.first, point2.second)) {
      return true;
    }

    // Compute the number of steps for intermediate points
    int steps_number = static_cast<int>(std::ceil(dist / resolution_));
    float theta = atan2(point2.second - point1.second, point2.first - point1.first);

    // Sample intermediate points along the line
    std::pair<float, float> p_n;
    for (int n = 1; n < steps_number; n++) {
      p_n.first = point1.first + n * resolution_ * cos(theta);
      p_n.second = point1.second + n * resolution_ * sin(theta);

      // Check if the intermediate point collides
      if (isThisPointCollides(p_n.first, p_n.second)) {
        return true;
      }
    }

    // No collision detected
    return false;
  }
}

// Collision test between two nodes
bool CollisionDetector::isThereObstacleBetween(const Node &node1, const Node &node2) {
  return isThereObstacleBetween(node1, std::make_pair(node2.x, node2.y));
}

// void CollisionDetector::worldToMap(float wx, float wy, int& mx, int& my) {
//   if (costmap_ != nullptr) {
//     mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
//     my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
//   }
// }

std::pair<std::pair<double, double>, std::pair<double, double>> CollisionDetector::getMapBounds() const {
  if (costmap_ == nullptr) {
    // Return zeros if costmap is not loaded
    return {{0.0, 0.0}, {0.0, 0.0}};
  }
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  double width = costmap_->getSizeInCellsX() * costmap_->getResolution();
  double height = costmap_->getSizeInCellsY() * costmap_->getResolution();

  // Bottom left: (origin_x, origin_y)
  // Upper right: (origin_x + width, origin_y + height)
  return {{origin_x, origin_y}, {origin_x + width, origin_y + height}};
}

}  // namespace glwoa_rrtstar_planner
