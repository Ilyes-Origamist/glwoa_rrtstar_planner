/* ******************************
  Copyright 2025 - Ilyes Chaabeni
 ****************************** */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// namespace glwoa_rrtstar_planner {

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_sender");
    ros::NodeHandle nh;

    ROS_INFO("Waiting 3 seconds before sending goal...");
    ros::Duration(3.0).sleep();

    // Create the action client (true = start a new thread)
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";  // Set to "odom" if needed
    goal.target_pose.header.stamp = ros::Time::now();

    double goal_x, goal_y;
    // Set goal coordinates
    nh.param("/move_base/SendGoal/goal_x", goal_x, 0.0);  // Default to 0.00
    nh.param("/move_base/SendGoal/goal_y", goal_y, 0.0);   // Default to 0.00
    // env1
    // goal.target_pose.pose.position.x = 0.205;
    // goal.target_pose.pose.position.y = 2.620;
    // goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending Goal: (%.3f, %.3f)", goal_x, goal_y);
    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;
    goal.target_pose.pose.orientation.w = 1.0;
    
    // env2
    // goal.target_pose.pose.position.x = 0.205;
    // goal.target_pose.pose.position.y = 2.620;
    // goal.target_pose.pose.orientation.w = 1.0;



    ROS_INFO("Sending goal to move_base...");
    ac.sendGoal(goal);

    ROS_INFO("Waiting for result...");
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Goal reached successfully!");
    else
        ROS_WARN("Failed to reach goal.");

    return 0;
}

