/**
 * Robotics control in a simulated environment
 * Robotics Engineering
 * 
 * @file final_robot.cpp
 * @author Simone Contorno (@simone-contorno)
 * 
 * @copyright Copyright (c) 2022
 */

// Headers
#include <iostream>
#include <string>
#include <chrono>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

// Declare the publishers
ros::Publisher pub_canc; 
ros::Publisher pub_vel;

float x_goal; // Current goal coordinate x
float y_goal; // Current goal coordinate y

std::string id = ""; // Goal ID
std::chrono::high_resolution_clock::time_point t_start;  
std::chrono::high_resolution_clock::time_point t_end; 

#define DIST 0.35 // Minimum distance from the wall with the driving assistance enabled
#define POS_ERROR 0.5 // Position range error
#define MAX_TIME 120000000 // Maximum time to reach a goal (microseconds)

/**
 * Check data from robot's laser scanner and,
 * if the driving assistance is enable, help the user to not crush 
 * the robot against a wall. 
 * 
 * @param msg 
 */
void drivingAssistance(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Local variables
    geometry_msgs::Twist robot_vel;
    float left = 30.0;
    float mid = 30.0;
    float right = 30.0;
    int i;
    int drive_flag = 0; // Enable/Disable driving assistance
    int goal_flag = 0; // Compute the time elapsed since the request of the current goal
    int print_flag = 0; // Just to manage printing
    int key_flag = 3; // Check the current key choosen by the user

    // Take value in the parameters server
    if (ros::param::has("/key_flag")) {
        ros::param::get("/key_flag", key_flag);
    }
    if (ros::param::has("/drive_flag")) {
        ros::param::get("/drive_flag", drive_flag);
    }
    if (ros::param::has("/goal_flag")) {
        ros::param::get("/goal_flag", goal_flag);
    }
    if (ros::param::has("/print_flag")) {
        ros::param::get("/print_flag", print_flag);
    }

    // Take the minimum values
    for (i = 0; i < 360; i++) { // On the right
        if (msg->ranges[i] < right)
            right = msg->ranges[i];
    }
    for (i = 300; i < 420; i++) { // In the middle
        if (msg->ranges[i] < mid)
            mid = msg->ranges[i];
    }
    for (i = 360; i < 720; i++) { // On the left
        if (msg->ranges[i] < left)
            left = msg->ranges[i];
    }

    // Driving assistance
    if (drive_flag == 1 & ((mid < DIST && key_flag == 0) || (left < DIST && key_flag == 1) || (right < DIST && key_flag == 2))) {
        if (print_flag == 0) {
            printf("\nThe robot is too close to the wall!\n");
            ros::param::set("/print_flag", 1);
        }
        robot_vel.linear.x = 0;
        robot_vel.angular.z = 0;
        pub_vel.publish(robot_vel);
    }

    // Check for the max time available to reach a goal point
    if (goal_flag == 1) {
        t_end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        if (time > MAX_TIME) {
            actionlib_msgs::GoalID canc_goal;
            printf("\nThe goal point can't be reached!\n");
            canc_goal.id = id;
            pub_canc.publish(canc_goal);
            printf("Goal cancelled.\n");
            ros::param::set("/goal_flag", 0);
        }
    }   
}

/**
 * Check if the robot is on the goal position and,
 * when there is a new goal, update the current Goal ID and save its in a 
 * global variable.
 * 
 * @param msg 
 */
void currentStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
    // Take the current robot position
    float diff_x;
    float diff_y;
    float current_x = msg->feedback.base_position.pose.position.x;
    float current_y = msg->feedback.base_position.pose.position.y;

    // Take the module
    if (current_x < 0)
        current_x *= -1;
    if (current_y < 0)
        current_y *= -1;
    
    // Compute the error from the actual position and the goal position
    if (current_x >= x_goal)
        diff_x = current_x - x_goal;
    else 
        diff_x = x_goal - current_x;
    if (current_y >= y_goal)
        diff_y = current_y - y_goal;
    else 
        diff_y = y_goal - current_y;

    // The robot is on the goal position
    if (diff_x <= POS_ERROR && diff_y <= POS_ERROR) 
        ros::param::set("/goal_flag", 0);

    // Update the goal ID if there is a new goal
    if (id != msg->status.goal_id.id) {
        printf("\nNew goal registered.\n");
        id = msg->status.goal_id.id;
        t_start = std::chrono::high_resolution_clock::now();
    }
}

/**
 * Check the current goal position and saves its 
 * module coordinates x and y in two global variables.
 * 
 * @param msg 
 */
void currentGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg) {
    x_goal = msg->goal.target_pose.pose.position.x;
    y_goal = msg->goal.target_pose.pose.position.y;

    // Take the module
    if (x_goal < 0)
        x_goal *= -1;
    if (y_goal < 0)
        y_goal *= -1;
}

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "final_robot");
    ros::NodeHandle nh;
    
    t_start = std::chrono::high_resolution_clock::now();

    // Define the publishers
    pub_canc = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1000);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    // Define the subscribers
    ros::Subscriber sub_pos = nh.subscribe("/move_base/feedback", 1000, currentStatus); // Current  Status feedback
    ros::Subscriber sub_goal = nh.subscribe("/move_base/goal", 1000, currentGoal); // Current Goal feedback
    ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, drivingAssistance); // Laser scanner

    // Multi-threading
    ros::AsyncSpinner spinner(3);
    spinner.start();
    char any;
    printf("If you want to quit, enter any key: ");
    std::cin >> any;
    printf("\nBye.\n");
    spinner.stop();
    ros::shutdown();
    ros::waitForShutdown();

    return 0;
}