/* ******************************
  Copyright 2025 - Ilyes Chaabeni
  Copyright 2021 - Rafael Barreto
 ****************************** */

#ifndef GLWOA_RRTSTAR_PLANNER_GLWOA_RRTSTAR_PLANNER_HPP_  // NOLINT
#define GLWOA_RRTSTAR_PLANNER_GLWOA_RRTSTAR_PLANNER_HPP_

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <vector>
#include <list>
#include <utility>
#include <memory>
#include <armadillo> 

#include "glwoa_rrtstar_planner/node.hpp"
#include "glwoa_rrtstar_planner/rrt_star.hpp"
#include "glwoa_rrtstar_planner/random_double_generator.hpp"
#include "glwoa_rrtstar_planner/random_int_generator.hpp"

#include <dynamic_reconfigure/server.h>
#include <glwoa_rrtstar_planner/GlWoaRrtStarPlannerConfig.h>

namespace glwoa_rrtstar_planner {

/**
 * @class GlWoaRrtStarPlanner
 * @brief Provides a ROS global planner plugin for the GLWOA-RRT* algorithm.
 */
class GlWoaRrtStarPlanner : public nav_core::BaseGlobalPlanner {
 public:
  /**
   * @brief Default constructor for the GlWoaRrtStarPlanner object
   */
  GlWoaRrtStarPlanner();

  /**
   * @brief  Constructor for the GlWoaRrtStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use
   */
  GlWoaRrtStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Constructor for the GlWoaRrtStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use
   * @param  global_frame The global frame of the costmap
   */
  GlWoaRrtStarPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief  Initialization function for the GlWoaRrtStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the GlWoaRrtStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use for planning
   * @param  global_frame The global frame of the costmap
   */
  void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief Given a start and goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);  // NOLINT

  /** 
  * @brief Converts a list of points to a ROS plan and publishes it using a ROS publisher object
  * @param plan The vector to store the converted poses
  * @param path The list of points representing the initial path
  * @param publisher The ROS publisher object that publishes the path
  **/
  void publishRosPlan(std::vector<geometry_msgs::PoseStamped>& plan,  // NOLINT
                      const std::list<std::pair<float, float>> &path,
                      const ros::Publisher& publisher);

  /**
   * @brief Optimizes the path using Whale Optimization Algorithm (WOA)
   * @param path The path to be optimized
   * @param N The maximum number of iterations for WOA
   * @param Ng The number of agents for WOA
   * @param spiral_shape The spiral shaping parameter (b) for WOA
   */
  void woaOptimizePath(std::list<std::pair<float, float>> &path, int N, int Ng, float spiral_shape);

  /**
   * @brief Callback for dynamic reconfigure server (get parameters in real time)
   * @param config The configuration parameters
   * @param level The level of the configuration change
   */
  void reconfigureCallback(GlWoaRrtStarPlannerConfig& config, uint32_t level);

  /**
   * @brief Converts a path agent to a list of points (path)
   * @param agent The agent vector 
   * @param path The converted path (it will be modified)
   */  
  void agentToPath(const arma::vec &agent, std::list<std::pair<float, float>> &path);

  /**
   * @brief Publishes the tree as Marker message in Rviz
   * @param nodes a vector of nodes containing the tree 
   */  
  void displayTree(const std::vector<Node> &nodes);

  /**
   * @brief Publishes the tree as Marker message in Rviz
   * @param nodes a vector of nodes containing the tree 
   * @param initial_path a list of points containing the initial path
   * @param final_path a list of points containing the final path
   */  
  void displayTree(const std::vector<Node> &nodes, 
                   const std::list<std::pair<float, float>> &initial_path, 
                   const std::list<std::pair<float, float>> &final_path);

 private:

   /**
   * @brief Helper function to create a basic marker with common properties
   * @param id Marker ID
   * @param ns Marker namespace
   * @param line_width Line width for the marker
   * @param color Color for the marker
   * @return Configured marker with common properties
   */
  visualization_msgs::Marker createBaseMarker(int id, const std::string& ns, 
                                              double line_width, 
                                              const std_msgs::ColorRGBA& color) const;

  /**
   * @brief Helper function to add line segments from a path to a marker
   * @param marker Marker to add points to
   * @param path Path containing the points
   */
  void addPathToMarker(visualization_msgs::Marker& marker, 
                       const std::list<std::pair<float, float>>& path) const;

  /**
   * @brief Helper function to add tree edges to a marker
   * @param marker Marker to add points to
   * @param nodes Vector of nodes representing the tree
   */
  void addTreeToMarker(visualization_msgs::Marker& marker, 
                       const std::vector<Node>& nodes) const;

 private:
  /**
   * @brief Get the cost of the best agent's path found by GLWOA-RRT*
   * @return The cost of the best path
   */
  float getPathCost() const {
    return best_cost_;
  }
  
  // ROS publisher objects
  ros::Publisher path_pub_; // for publishing the final optimized path found by GLWO-RRT*
  ros::Publisher initial_path_pub_; // for publishing the initial path found by RRT*
  ros::Publisher tree_pub_; // RRT* tree publisher (ROS marker) for visualization

  double best_cost_{0.0}; // the cost of the best path
  costmap_2d::Costmap2D* costmap_{nullptr}; // pointer to the ROS costmap used for collision checks
  bool initialized_{false}; // whether the global planner has been initialized or not yet 

  // general planner parameters
  bool visualize_tree_{true}; // whether to visualize RRT* tree (and paths)
  bool use_optimizer_{true}; // whether to use WOA (GLWOA-RRT*) or only RRT*
  int N_tests_{1}; // number of runs for the GLWOA-RRT* planner
  float map_width_;
  float map_height_;
  double goal_tolerance_; 
  std::string global_frame_;

  // parameters for RRT*
  std::shared_ptr<RRTStar> planner_; // pointer to the RRT* planner object
  double epsilon_; // step size for RRT*
  double rewiring_radius_; // rewiring radius for RRT*
  int max_num_nodes_; // maximum number of nodes in the RRT* tree
  int min_num_nodes_; // minimum number of nodes in the RRT* tree
  
  // parameters for WOA
  int N_; // max number of iterations for WOA
  int Ng_; // number of agents for WOA
  float b_; // spiral shaping parameter for WOA
  double sampling_radius_; // sampling radius for GLWOA-RRT* initialization
  // Random devices for WOA
  RandomDoubleGenerator rand_double; // Random double generator
  RandomIntGenerator rand_index; // Random integer generator
  uint16_t agent_size_; // size of a single agent

  // goal orientation
  float goal_z{0.0};
  float goal_w{1.0};

  // rqt reconfigure
  typedef dynamic_reconfigure::Server<glwoa_rrtstar_planner::GlWoaRrtStarPlannerConfig> drs;
  // dynamic reconfigure server ptr
  boost::shared_ptr<drs> dr_server_;};

  // Visualization constants
  static constexpr int TREE_MARKER_ID = 0;
  static constexpr int INITIAL_PATH_MARKER_ID = 1;
  static constexpr int FINAL_PATH_MARKER_ID = 2;
  
  static constexpr double TREE_LINE_WIDTH = 0.02;
  static constexpr double INITIAL_PATH_LINE_WIDTH = 0.06;
  static constexpr double FINAL_PATH_LINE_WIDTH = 0.04;

  static const std::vector<float> TREE_COLOR = {1.0f, 0.0f, 1.0f}; // Magenta
  static const std::vector<float> INITIAL_PATH_COLOR = {1.0f, 0.0f, 0.0f}; // Red
  static const std::vector<float> FINAL_PATH_COLOR = {0.0f, 0.0f, 1.0f}; // Blue
}  // namespace glwoa_rrtstar_planner

#endif  // GLWOA_RRTSTAR_PLANNER_GLWOA_RRTSTAR_PLANNER_HPP_  // NOLINT
