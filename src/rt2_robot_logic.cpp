/**
 * @file rt2_robot_logic.cpp
 * @author Simone Contorno (simone.contorno@outlook.it)
 * @brief Robotics control in a simulated environment
 * @version 1.0
 * @date 2022-03-25
 * 
 * @copyright Copyright (c) 2022
 * 
 * @details 
 * 
 * Server parameters: <br>
 * @param key_flag defines the type of keyboard input inserted by the user (output).
 * @param drive_flag defines if the manual driving assistance is enabled or not (output).
 * @param goal_flag defines if a goal is set or not (input & output).
 * @param print_flag defines if the warning sentence, about a wall too close to the robot, has been printed or not (input & output).
 * 
 * Publishers to: <br>
 * /move_base/cancel <br>
 * /cmd_vel
 * 
 * Subscribers to: <br>
 * /move_base/feedback <br>
 * /move_base/goal <br>
 * /scan
 * 
 * Description: <br>
 * This node simulates the 'logic' of the robot. <br>
 * Managing publishers, subscribers and server parameters it is able to avoid the crashing of 
 * the robot against a wall, if asked by the user, and register how many targets
 * it reached and how many not. <br>
 * A target is considered 'not reached' when the robot does not reach it within 2 minutes, then the goal is cancelled.
 * 
 */

// Headers
#include <iostream>
#include <string>
#include <chrono>
#include <ctime>

// ROS headers
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

// Publishers
ros::Publisher pub_canc; ///< Publishes to /move_base/cancel
ros::Publisher pub_vel; ///< Publishes to /cmd_vel

// Global variables
float x_goal; ///< Current goal coordinate x
float y_goal; ///< Current goal coordinate y
int reached_goal = 0; ///< Number of reached goals
int non_reached_goal = 0; ///< Number of not reached goals

std::string id = ""; ///< Goal ID
std::chrono::high_resolution_clock::time_point t_start; ///< Starting time goal reaching
std::chrono::high_resolution_clock::time_point t_end; ///< Ending time goal reaching

#define DIST 0.35 ///< Minimum distance from the wall with the driving assistance enabled
#define POS_ERROR 0.5 ///< Position range error
#define MAX_TIME 120000000 ///< Maximum time to reach a goal (microseconds)

/**
 * @brief Check data from robot's laser scanner and, 
 * if the driving assistance is enable, help the user not to crush the robot against a wall. 
 * 
 * @param msg defines the laser scanner values.
 */
void drivingAssistance(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Local variables
    geometry_msgs::Twist robot_vel;
    float left = 30.0;
    float mid = 30.0;
    float right = 30.0;
    int i;
    int drive_flag; // Enable/Disable driving assistance
    int goal_flag; // Compute the time elapsed since the request of the current goal
    int print_flag; // Just to manage printing
    int key_flag; // Check the current key choosen by the user
    std::time_t t = std::time(0); 
    std::tm* time_info = std::localtime(&t);

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
            printf("[%d-%d-%d %d:%d:%d] The robot is too close to the wall!\n", time_info->tm_mday, time_info->tm_mon + 1, time_info->tm_year + 1900, 
            time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
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
            printf("[%d-%d-%d %d:%d:%d] The goal point can't be reached!\n", time_info->tm_mday, time_info->tm_mon + 1, time_info->tm_year + 1900, 
            time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
            canc_goal.id = id;
            pub_canc.publish(canc_goal);
            printf("[%d-%d-%d %d:%d:%d] Goal cancelled\n", time_info->tm_mday, time_info->tm_mon + 1, time_info->tm_year + 1900, 
            time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
            non_reached_goal++;
            ros::param::set("/non_reached_goal", non_reached_goal);
            ros::param::set("/goal_flag", 0);
        }
    }
    else if (goal_flag == 0) {
        actionlib_msgs::GoalID canc_goal;
        canc_goal.id = id;
        pub_canc.publish(canc_goal);
    }  
}

/**
 * @brief Check if the robot is on the goal position and, when there is a new goal, 
 * update the current Goal ID and save its in a global variable.
 * 
 * @param msg defines the robot position values.
 */
void currentStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
    // Take the current robot position
    int goal_flag;
    float diff_x;
    float diff_y;
    float current_x = msg->feedback.base_position.pose.position.x;
    float current_y = msg->feedback.base_position.pose.position.y;
    std::time_t t = std::time(0); 
    std::tm* time_info = std::localtime(&t);

    if (ros::param::has("/goal_flag")) {
        ros::param::get("/goal_flag", goal_flag);
    }

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
    if (diff_x <= POS_ERROR && diff_y <= POS_ERROR) {        
        if (goal_flag == 1)
            printf("[%d-%d-%d %d:%d:%d] Goal reached\n", time_info->tm_mday, time_info->tm_mon + 1, time_info->tm_year + 1900, 
            time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
        reached_goal++;
        ros::param::set("/reached_goal", reached_goal);
        ros::param::set("/goal_flag", 0);
    }

    // Update the goal ID if there is a new goal
    if (id != msg->status.goal_id.id) {
        printf("[%d-%d-%d %d:%d:%d] New goal registered\n", time_info->tm_mday, time_info->tm_mon + 1, time_info->tm_year + 1900, 
            time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
        ros::param::set("/goal_flag", 1);
        id = msg->status.goal_id.id;
        t_start = std::chrono::high_resolution_clock::now();
    }
}

/**
 * @brief Check the current goal position and saves its module coordinates x and y in two global variables.
 * 
 * @param msg defines the goal position values.
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

/**
 * @brief Initializes the node and starts the logic of the robot.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "rt2_robot_logic");
    ros::NodeHandle nh;
    
    t_start = std::chrono::high_resolution_clock::now();

    // Define the publishers
    pub_canc = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1000);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    // Define the subscribers
    ros::Subscriber sub_pos = nh.subscribe("/move_base/feedback", 1000, currentStatus); /// Subscriber on /move_base/feedback
    ros::Subscriber sub_goal = nh.subscribe("/move_base/goal", 1000, currentGoal); /// Subscriber on /move_base/goal
    ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, drivingAssistance); /// Subscriber on /scan

    // Set number of reached and non-reached goal in the parameter server
    ros::param::set("/reached_goal", 0);
    ros::param::set("/non_reached_goal", 0);

    // Multi-threading
    ros::AsyncSpinner spinner(3);
    spinner.start();
    char any;
    printf("If you want to quit, enter any key:\n");
    std::cin >> any;
    printf("\nBye.\n");
    spinner.stop();
    ros::shutdown();
    ros::waitForShutdown();

    return 0;
}