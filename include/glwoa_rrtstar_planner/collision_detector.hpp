/* ******************************
  Copyright 2025 - Ilyes Chaabeni
  Copyright 2021 - Rafael Barreto
 ****************************** */

#ifndef GLWOA_RRTSTAR_PLANNER_COLLISION_DETECTOR_HPP_  // NOLINT
#define GLWOA_RRTSTAR_PLANNER_COLLISION_DETECTOR_HPP_

#include <costmap_2d/costmap_2d.h>
#include <utility>
#include "glwoa_rrtstar_planner/node.hpp"

namespace glwoa_rrtstar_planner {

class CollisionDetector {
 public:
  explicit CollisionDetector(costmap_2d::Costmap2D* costmap);

  bool isThisPointCollides(float wx, float wy);

  // Collision test between node and point
  bool isThereObstacleBetween(const Node &node, const std::pair<double, double> &point);

  // Collision test between two points
  bool isThereObstacleBetween(const std::pair<double, double> &point1, const std::pair<double, double> &point2);

  // Collision test two nodes
  bool isThereObstacleBetween(const Node &node1, const Node &node2);

  // void worldToMap(float wx, float wy, int& mx, int& my);  // NOLINT

  std::pair<std::pair<double, double>, std::pair<double, double>> getMapBounds() const;

 private:
  costmap_2d::Costmap2D* costmap_{nullptr};
  double resolution_{0.1};
  double precision_{1.0}; // resolution is divided by this value to increase precision
  double origin_x_{0.0};
  double origin_y_{0.0};
};

}  // namespace glwoa_rrtstar_planner

#endif  // GLWOA_RRTSTAR_PLANNER_COLLISION_DETECTOR_HPP_  NOLINT
