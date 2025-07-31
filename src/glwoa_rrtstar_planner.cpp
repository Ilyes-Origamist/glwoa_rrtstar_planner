/* ******************************
  Copyright 2025 - Ilyes Chaabeni
  Copyright 2021 - Rafael Barreto
 ****************************** */

#include <pluginlib/class_list_macros.h>
#include "glwoa_rrtstar_planner/glwoa_rrtstar_planner.hpp"
#include "glwoa_rrtstar_planner/woa_agent.hpp"
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <cmath>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(glwoa_rrtstar_planner::GlWoaRrtStarPlanner, nav_core::BaseGlobalPlanner)

namespace glwoa_rrtstar_planner {

// ===============================
//   Constructor of the planner
// ===============================

GlWoaRrtStarPlanner::GlWoaRrtStarPlanner() : costmap_(nullptr), initialized_(false) {}

GlWoaRrtStarPlanner::GlWoaRrtStarPlanner(std::string name,
                               costmap_2d::Costmap2DROS* costmap_ros) : costmap_(nullptr), initialized_(false) {
  // initialize the planner
  initialize(name, costmap_ros);
}

GlWoaRrtStarPlanner::GlWoaRrtStarPlanner(std::string name,
                               costmap_2d::Costmap2D* costmap,
                               std::string global_frame) : costmap_(nullptr), initialized_(false) {
  // initialize the planner
  initialize(name, costmap, global_frame);
}

// =================================
//     Initialization Functions
// =================================

// Initialize(name, costmap)

void GlWoaRrtStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

// Initialize(name, costmap, global_frame)

void GlWoaRrtStarPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) {
  if (!initialized_) {
    costmap_ = costmap;
    global_frame_ = global_frame;

    ros::NodeHandle private_nh("~/" + name);
    // initialize path publishers
    path_pub_ = private_nh.advertise<nav_msgs::Path>("/move_base/GlWoaRrtStarPlanner/global_plan", 1, true);
    initial_path_pub_ = private_nh.advertise<nav_msgs::Path>("/move_base/GlWoaRrtStarPlanner/initial_plan", 1, true);
    tree_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("/move_base/GlWoaRrtStarPlanner/tree", 1, true);
    
    // Dynamic reconfigure
    dr_server_.reset(new drs(private_nh));
    drs::CallbackType f = boost::bind(&GlWoaRrtStarPlanner::reconfigureCallback, this, _1, _2);
    dr_server_->setCallback(f);
  
    // map origin and dimensions*
    std::pair<float, float> map_origin;
    map_origin = {static_cast<float>(costmap_->getOriginX()),
                   static_cast<float>(costmap_->getOriginY())};
    ROS_INFO("Map origin: %f m, %f m", map_origin.first, map_origin.second);
    map_width_ = costmap_->getSizeInMetersX();
    map_height_ = costmap_->getSizeInMetersY();
    ROS_INFO("Map dimensions: %f m, %f m", map_width_, map_height_);
    // initialize random number generators (random devices) for WOA
    rand_double.setRangeFirst(0,1); // used for r and p
    rand_double.setRangeSecond(-1,1); // used for l
    rand_index.setRange(0,Ng_-1); // used for selecting agents

    ROS_INFO("GLWOA-RRT* Global Planner initialized successfully.");
    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized... doing nothing.");
  }
}

// ====================================
//    Dynamic reconfigure Callback
// ====================================

void GlWoaRrtStarPlanner::reconfigureCallback(GlWoaRrtStarPlannerConfig& config, uint32_t level){
  // General parameters
  visualize_tree_ = config.visualize_tree;
  use_optimizer_ = config.use_optimizer;
  N_tests_ = config.number_of_tests;
  // RRT* parameters
  goal_tolerance_= config.goal_tolerance;
  rewiring_radius_= config.rewiring_radius;
  epsilon_= config.epsilon;
  max_num_nodes_= config.max_num_nodes;
  // WOA parameters
  sampling_radius_= config.sampling_radius;
  N_= config.num_iterations;
  Ng_= config.num_agents;
  b_= config.spiral_shape;
  // Logs
  // ROS_INFO("Dynamic Reconfigure: Parameters updated!");
}

// ====================================
//   makePlan Function (main function)
// ====================================

bool GlWoaRrtStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan) {

  ROS_INFO("GLWOA-RRT* Global Planner");
  ROS_INFO("Current Position: ( %.2lf, %.2lf)", start.pose.position.x, start.pose.position.y);
  ROS_INFO("GOAL Position: ( %.2lf, %.2lf)", goal.pose.position.x, goal.pose.position.y);
  std::pair<float, float> start_point = {start.pose.position.x, start.pose.position.y};
  std::pair<float, float> goal_point = {goal.pose.position.x, goal.pose.position.y};
  goal_z = goal.pose.orientation.z;
  goal_w = goal.pose.orientation.w;

  // Multiple tests version
  float sum_time = 0.0, sum_time_initial = 0.0;
  float sum_cost = 0.0, sum_cost_initial = 0.0;
  int n_fails = 0;

  ROS_INFO("---> Running GLWOA-RRT* Planner with %d agents for %d iterations", Ng_, N_);
  if (N_tests_ > 1)
    ROS_INFO("---------------Performing %d tests", N_tests_);

  std::list<std::pair<float, float>> best_path;
  float best_path_cost = FLT_MAX, best_initial_path_cost = FLT_MAX;
  float path_cost = FLT_MAX;
  std::list<std::pair<float, float>> worst_path, worst_initial_path;
  float worst_path_cost = 0.0, worst_initial_path_cost = 0.0;
  std::vector<Node> best_path_nodes;

  /*********** Loop for multiple tests ********** */ 

  for(int nn=1; nn<=N_tests_; nn++){
    if(N_tests_ > 1)
      ROS_INFO("------- Performing test %d", nn);

    // create a new RRT* planner instance
    planner_ = std::shared_ptr<RRTStar>(new RRTStar(start_point,
      goal_point,
      costmap_,
      goal_tolerance_,
      rewiring_radius_,
      epsilon_,
      max_num_nodes_,
      min_num_nodes_));

    std::list<std::pair<float, float>> path;
    std::list<std::pair<float, float>> rand_path;
    std::list<std::pair<float, float>> initial_path;

    // clear the previous plan
    plan.clear();
    // start measuring time for the RRT* path computation
    auto start_time = std::chrono::steady_clock::now();
    ROS_INFO("Started computing path with RRT*");

    if (planner_->initialPath(path)) {
      // evaluate initial planning time
      auto end_time_1 = std::chrono::steady_clock::now();
      auto diff_1 = std::chrono::duration<double, std::milli>(end_time_1 - start_time);
      auto total_planning_time_initial = diff_1.count();
      initial_path = path;
      float initial_path_cost = planner_->getPathCost();
      sum_cost_initial += initial_path_cost;
      sum_time_initial += total_planning_time_initial;

      // update best/worst path
      if (initial_path_cost < best_initial_path_cost){
        best_initial_path_cost = initial_path_cost;
        best_path = path;
      }
      if (initial_path_cost > worst_initial_path_cost){
        worst_initial_path_cost = initial_path_cost;
      }

    /*********** WOA  Path  Optimization ********** */ 

    // optimize path only if path has more than 2 points
    // and if the optimizer is enabled
    if (path.size()>2 && use_optimizer_) {
      ROS_INFO("Proceeding to path optimization with WOA.");
      // optimize the path using WOA
      woaOptimizePath(path, N_, Ng_, b_);
      path_cost = best_cost_;  // Use the WOA-optimized cost

      // evaluate planning time
      auto end_time = std::chrono::steady_clock::now();
      auto diff = std::chrono::duration<double, std::milli>(end_time - start_time);
      auto total_planning_time = diff.count();
      ROS_INFO("Total planning time: %.2f ms", total_planning_time);
      
      // update average time and cost
      sum_time += total_planning_time;
      sum_cost += path_cost;

      // update best/worst path
      if (path_cost < best_path_cost){
        best_path_cost = path_cost;
        // best_path.clear();
        best_path = path;
        best_path_nodes = planner_->getNodes();
      }
      if (path_cost > worst_path_cost){
        worst_path_cost = path_cost;
        worst_path = path;
      }
    }
    else if (!use_optimizer_) {
      ROS_INFO("WOA optimization disabled. Using RRT* path directly.");
    }
    else {
      ROS_WARN("Path contains only two points. No WOA optimization.");
      n_fails = n_fails + 1;
    }

    // Visualization (works for both optimized and non-optimized paths)
    if (visualize_tree_){
      std::vector<Node> nodes_vector = planner_->getNodes();
      if (use_optimizer_) {
        displayTree(nodes_vector, initial_path, path);
      } else {
        displayTree(nodes_vector, initial_path, {}); // Show initial path only
      }
    }

    // Publish paths (always publish both initial and final)
    publishRosPlan(plan, initial_path, initial_path_pub_);
    publishRosPlan(plan, path, path_pub_);
  }
  
  // if no initial path was found
  else {
    ROS_WARN("The planner failed to find a path. Please restart or choose another goal position.");
    n_fails = n_fails + 1;
    // visualize the tree despite the failure 
    // to show how far the tree expanded (debugging)
    if (visualize_tree_){
      // display the tree
      std::vector<Node> nodes_vector = planner_->getNodes();
      displayTree(nodes_vector);
    }
  }
}
/*********** end for loop (tests) ********** */

  if (N_tests_ > 1){
    /* Display statistics of the tests' results */
    if(n_fails == N_tests_){
        ROS_WARN("The planner failed to find a path on all tests.");
        return false;
    }
    else{
        if(n_fails > 0){ROS_WARN("The planner failed %d times", n_fails);}
        int total_tests = N_tests_ - n_fails;
        ROS_INFO("------------ Finished %d successful tests", total_tests);
        // update average time and cost
        if (use_optimizer_){
          ROS_INFO("Average planning time: %.2f ms", sum_time / total_tests);
          ROS_INFO("Average initial planning time: %.2f ms", sum_time_initial / total_tests); 
          ROS_INFO("Average path cost: %.4f m", sum_cost / total_tests);
          ROS_INFO("Average initial path cost: %.4f m", sum_cost_initial / total_tests);
          ROS_INFO("Best path cost: %.4f", best_path_cost);
          ROS_INFO("Best initial path cost: %.4f", best_initial_path_cost);
          ROS_INFO("Worst path cost: %.4f", worst_path_cost);
          ROS_INFO("Worst initial path cost: %.4f", worst_initial_path_cost);
        }
        else{ 
          ROS_INFO("Average RRT* planning time: %.2f ms", sum_time_initial / total_tests); 
          ROS_INFO("Average RRT* path cost: %.4f m", sum_cost_initial / total_tests);
          ROS_INFO("Best RRT* path cost: %.4f", best_initial_path_cost);
          ROS_INFO("Worst RRT* path cost: %.4f", worst_initial_path_cost);
        }
        // use the best path
        publishRosPlan(plan, best_path, path_pub_);
        std::list<std::pair<float, float>> initial_path = {{0.0, 0.0}};
        if (visualize_tree_){
          // display the tree
          displayTree(best_path_nodes, initial_path, best_path);
        }
    }
  }
  return !plan.empty();
  // return true;
}

// =================================
//           WOA Algorithm
// =================================

void GlWoaRrtStarPlanner::woaOptimizePath(std::list<std::pair<float, float>> &path, int N, int Ng, float spiral_shape) {
  // Initialization
  //--------------- IMPORTANT NOTE --------------------
  // A vector of pointers to all the agents is used
  // A pointer is used instead of object to avoid moving the object PathAgent
  // which will make errors since RandomDoubleGenerator is non-movable
  // --------------------------------------------------
  std::vector<std::unique_ptr<PathAgent>> agents; 
  std::vector<geometry_msgs::PoseStamped> plan; 
  // initial path
  std::list<std::pair<float, float>> initial_path;
  initial_path=path;
  // start chronometer
  auto start_time = std::chrono::steady_clock::now();

  // initialize each agent
  for (int i = 0; i < Ng; ++i) {
    agents.emplace_back(std::make_unique<PathAgent>(initial_path, sampling_radius_, i, costmap_, spiral_shape));
    // constructor: object_name(path, sampling_radius, id, costmap_ptr, b)

    // -------------- debugging --------------
    // display the initial path for each agent
    // ---------------------------------------
    // publishRosPlan(plan, agents[i]->initial_path_, path_pub_);
    // ROS_INFO("Displaying %d-th agent's initial path.", i+1);
    // ros::Duration(0.25).sleep(); 
  }
  agent_size_= agents[0]->vec_size;

  // commented is the second version of the WOA

  // Random variables
  double p, l, r, a;
  // float p, l, r1, r2, a, a2;

  double A, C;
  int rand; // random agent index

  // initialize Xbest with RRT* initial path (Agent 0)
  arma::vec Xbest;
  Xbest.set_size(agent_size_);
  Xbest = agents[0]->X;
  // initialize best cost
  best_cost_ = agents[0]->fitness();

  double fitness = 0.0;
  // // for debugging
  // double collision_rate = 0.0;
  // double out_of_bounds_rate = 0.0;

  /*********** Main loop ********** */
  
  for (int t=0; t<N; t++){
    // iterate over all agents fo find the best solution
    for (size_t i = 0; i < Ng; ++i) {
      // Update Xbest
      fitness=agents[i]->fitness(); // cost(Xi)
      if (fitness < best_cost_){
        // update Xbest if there is a better solution that does not collide
        if (!agents[i]->doesPathCollide()){
          Xbest=agents[i]->X;
          best_cost_=fitness;
          // ROS_INFO("Current Best cost (iteration %d): %.6f", t, best_cost_);
        }
      }
      // // for debugging
      // if (agents[i]->doesPathCollide()){
      //   collision_rate+=100.0/(N*Ng);
      //   // ROS_INFO("Collision rate: %.4f", collision_rate);
      // }
    }
 
    // update parameter a
    // a decreases linearly fron 2 to 0 in Eq. (2.3)
    a=2-2*t/N;
    // // for the 2nd version of WOA
    // // a2 linearly dicreases from -1 to -2 to calculate t in Eq. (3.12)
    // a2=-1-t/N;

    /*********** Iterate over each agent ********** */
    for (int i = 0; i < Ng; ++i) {
      // update the random variables
      p=rand_double.generateFirst(); // random number in [0,1]
      r=rand_double.generateFirst(); // random number in [0,1]

      // update A and C
      C=2*r;
      A=C*a-a;
      // // for the 2nd version of WOA
      // A=2*a*r1-a;  // Eq. (2.3) in the paper
      // C=2*r2;      // Eq. (2.4) in the paper

      // update the agent parameters A and C
      agents[i]->A=A;
      agents[i]->C=C;
      
      /* Circular Search */
      if (p<0.5){
        if (std::abs(A)>=1){
          /* Exploration */
          // select a random agent
          rand=rand_index.generateInt(); // random index
          arma::vec search_ag = agents[rand]->X;
          // Circular update
          agents[i]->circularUpdate(agents[rand]->X); // update Xi using Xrand 
        }
        else{
          /* Exploitation */
          agents[i]->circularUpdate(Xbest); // update Xi using Xbest 
        }
      }

      /* Spiral Search */
      else if (p>=0.5){
        l=rand_double.generateSecond();
        // // for the 2nd version of WOA
        // l=(a2-1)*rand_double.generateFirst()+1;   //  parameters in Eq. (2.5)

        agents[i]->l=l; // update l
        agents[i]->spiralUpdate(Xbest);
      }
      // check if the new path is out of bounds
      // if (agents[i]->clampToBounds()){
      //   out_of_bounds_rate+=100.0/(N*Ng);
      // }
    }

  }
  /*********** End of the Main loop ********** */

  // check the agents after the last iteration
  for (size_t i = 0; i < Ng; ++i) {
    // Get the best agent's path
    fitness=agents[i]->fitness(); // cost(Xi)
    if (fitness < best_cost_){
      // update Xbest if there is a better solution that does not collide
      if (!agents[i]->doesPathCollide()){
        Xbest=agents[i]->X;
        best_cost_=fitness;
        // ROS_INFO("Current Best cost (iteration %d): %.6f", t, best_cost_);
      }
    }
    // // for debugging
    // if (agents[i]->doesPathCollide()){
    //    collision_rate+=100.0/(N*Ng);
    // }
    // if (agents[i]->clampToBounds()){
    //   out_of_bounds_rate+=100.0/(N*Ng);
    // }
  }

  // // Debugging logs
  // ROS_WARN("Collision rate: %.2f percent", collision_rate);
  // ROS_WARN("Out of bounds rate: %.2f percent", out_of_bounds_rate);
  // -----------------------------------------------------------------------

  // Convert the best agent's path to a list of points
  agentToPath(Xbest, path);
  // ROS_INFO("Path Length after WOA: %ld", path.size());
  ROS_INFO("Path Cost after WOA Optimization: %.6f", best_cost_);

  // stop measuring time for the WOA path computation
  auto end_time = std::chrono::steady_clock::now();
  auto diff = std::chrono::duration<double, std::milli>(end_time - start_time);
  ROS_INFO("---> Time taken to optimize path with WOA: %.2f ms", diff.count());
}


// ==========================
//        agentToPath 
// ==========================
// Converts the agent vector to a list of points (path)

void GlWoaRrtStarPlanner::agentToPath(const arma::vec &agent, 
                                      std::list<std::pair<float, float>> &path){
  auto it = path.begin();
  for (int i=0; i<agent_size_; i+=2){
      ++it;
      it->first=agent.at(i);
      it->second=agent.at(i+1);
  } 
  ++it;
}

// =============================
//        publishRosPlan 
// =============================

void GlWoaRrtStarPlanner::publishRosPlan(std::vector<geometry_msgs::PoseStamped>& plan,
                                       const std::list<std::pair<float, float>> &path,
                                       const ros::Publisher & pub) {
  // check if the path is empty
  if (path.empty()) {
    ROS_WARN("The path is empty, nothing to publish.");
    return;
  }

  // clear plan
  plan.clear();
  ros::Time plan_time = ros::Time::now();

  // convert points to poses
  for (const auto &point : path) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }
  // set the last pose orientation to the goal orientation
  geometry_msgs::PoseStamped &last_pose = plan.back();
  last_pose.pose.orientation.z = goal_z;
  last_pose.pose.orientation.w = goal_w;

  // Publish the path
  nav_msgs::Path path_msg;
  path_msg.header.stamp = plan_time;
  path_msg.header.frame_id = global_frame_;
  path_msg.poses = plan;
  pub.publish(path_msg);
}

// ===================================
//  Tree and Path Visualization (Rviz)
// ===================================

/* ***** createColor ***** */

namespace {
  // Helper function to create color
  std_msgs::ColorRGBA createColor(const std::vector<float>& rgb, float a = 1.0f) {
    std_msgs::ColorRGBA color;
    color.r = rgb[0];
    color.g = rgb[1];
    color.b = rgb[2];
    color.a = a;
    return color;
  }
}

/* ***** createBaseMarker ***** */

visualization_msgs::Marker GlWoaRrtStarPlanner::createBaseMarker(int id, const std::string& ns, 
                                                                 double line_width, 
                                                                 const std_msgs::ColorRGBA& color) const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = global_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = line_width;
  marker.color = color;
  
  // Set neutral orientation
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  
  return marker;
}

/* ***** addPathToMarker ***** */

void GlWoaRrtStarPlanner::addPathToMarker(visualization_msgs::Marker& marker, 
                                          const std::list<std::pair<float, float>>& path) const {
  if (path.size() < 2) {
    return; // Need at least 2 points to create lines
  }
  
  for (auto it = path.begin(); it != path.end(); ++it) {
    auto next_it = std::next(it);
    if (next_it != path.end()) {
      geometry_msgs::Point p1, p2;
      p1.x = it->first;
      p1.y = it->second;
      p1.z = 0.0;
      
      p2.x = next_it->first;
      p2.y = next_it->second;
      p2.z = 0.0;
      
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  }
}

// ***** addTreeToMarker ***** /

void GlWoaRrtStarPlanner::addTreeToMarker(visualization_msgs::Marker& marker, 
                                          const std::vector<Node>& nodes) const {
  for (const auto& node : nodes) {
    if (node.parent_id != -1 && node.parent_id < static_cast<int>(nodes.size())) {
      geometry_msgs::Point p1, p2;
      p1.x = node.x;
      p1.y = node.y;
      p1.z = 0.0;
      
      p2.x = nodes[node.parent_id].x;
      p2.y = nodes[node.parent_id].y;
      p2.z = 0.0;
      
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  }
}

// ***** displayTree with paths ***** /

void GlWoaRrtStarPlanner::displayTree(const std::vector<Node>& nodes,
                                      const std::list<std::pair<float, float>>& initial_path,
                                      const std::list<std::pair<float, float>>& final_path) {
  visualization_msgs::MarkerArray marker_array;
  
  // Create final path's marker if the path has sufficient points
  if (final_path.size() > 1) {
    auto final_marker = createBaseMarker(FINAL_PATH_MARKER_ID, "final_path", 
                                         FINAL_PATH_LINE_WIDTH, 
                                         createColor(FINAL_PATH_COLOR));
    addPathToMarker(final_marker, final_path);
    marker_array.markers.push_back(final_marker);
  }

  // Create initial path's marker if the path has sufficient points
  if (initial_path.size() > 1) {
    auto initial_marker = createBaseMarker(INITIAL_PATH_MARKER_ID, "initial_path", 
                                           INITIAL_PATH_LINE_WIDTH, 
                                           createColor(INITIAL_PATH_COLOR));
    addPathToMarker(initial_marker, initial_path);
    marker_array.markers.push_back(initial_marker);
  }
  
  // Create the tree's marker
  auto tree_marker = createBaseMarker(TREE_MARKER_ID, "tree", TREE_LINE_WIDTH, 
                                      createColor(TREE_COLOR));
  addTreeToMarker(tree_marker, nodes);
  marker_array.markers.push_back(tree_marker);
  
  // Publish all markers
  tree_pub_.publish(marker_array);
}

// ***** displayTree without paths (tree only) ***** /

void GlWoaRrtStarPlanner::displayTree(const std::vector<Node>& nodes) {
  visualization_msgs::MarkerArray marker_array;
  
  // Create tree marker
  auto tree_marker = createBaseMarker(TREE_MARKER_ID, "tree", TREE_LINE_WIDTH, 
                                      createColor(TREE_COLOR));
  addTreeToMarker(tree_marker, nodes);
  marker_array.markers.push_back(tree_marker);
  
  // Publish the marker
  tree_pub_.publish(marker_array);
}

}  // namespace glwoa_rrtstar_planner