## @package rt2_robot
# \file rt2_robot_interface.py
# \author Simone Contorno (simone.contorno@outlook.it)
# \brief Robotics control in a simulated environment
# \version 1.0
# \date 2022-03-25
# 
# \copyright Copyright (c) 2022
# 
# \details 
# 
# Server parameters: <br>
# param key_flag define the type of keyboard input insert by the user (input). <br>
# param drive_flag define if the manual driving assistance is enable or not (input). <br>
# param goal_flag define if a goal is set or not (input). <br>
# param print_flag define if the warning sentence, about a wall to close to the robot, has been printed or not (input). <br>
# 
# Publishers to: <br>
# /move_base/cancel <br>
# /cmd_vel
# 
# Subscribers to: <br>
# /move_base/feedback <br>
# /move_base/goal <br>
# /scan
# 
# Description: <br>
# This nodes simulate the 'logic' of the robot. <br>
# Managing publishers, subscribers and server parameters it is able to avoid the crashing of 
# the robot against a wall, if asked by the user, and register how many targets
# it reached and how many not. <br>
# A target is considered 'not reached' when the robot does not reach it within 2 minutes, then the goal is cancelled.
# 

# ROS headers
import rospy
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Global variables
## Manages the interface printing
counter1 = 10 

## Defines if the driving assistance is enabled or not
flag = 0 

# Manual driving
def manualDriving():
    '''
    Provides an interface to manually drive the robot, managing its linear and angular velocities.
    '''
    
    # Publisher declaration
    pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1000)
    robot_vel = Twist() 
    
    global flag 
    global counter1
    lin_vel = 0.0 # Robot linear velocity
    ang_vel = 0.0 # Robot angular velocity
    counter2 = 10
    
    key = 'e'
    
    while key != 'f':
        # Command list
        if counter2 % 10 == 0:
            print("\nCommands:\n"
                "w - Go on\n"
                "s - Go back\n"
                "q - Curve left\n"
                "e - Curve right\n"
                "a - Turn left\n"
                "d - Turn right\n"
                "-----------------------------\n"
                "z - Increase linear velocity\n"
                "x - Decrease linear velocity\n"
                "c - Increase angular velocity\n"
                "v - Decrease angular velocity\n"
                "-----------------------------\n"
                "r - Emergency stop\n"
                "f - Quit")
            
        if flag == 0:
            print("h - Enable driving assistance")
        elif flag == 1:
            print("h - Disable driving assistance")
        
        # Take user input
        key = input("\nCommand: ")
        
        rospy.set_param('/print_flag', 0)
        
        for k in key:
            if k == 'z': # Increase linear velocity
                lin_vel += 0.1
            elif k == 'x': # Decrease linear velocity
                lin_vel -= 0.1
            elif k == 'c': # Increase angular velocity
                ang_vel += 0.1
            elif k == 'v': # Decrease angular velocity
                ang_vel -= 0.1
        if key == 'w': # Go on
            robot_vel.linear.x = lin_vel
            robot_vel.angular.z = 0 
        elif key == 'q': # Curve left
            robot_vel.linear.x = lin_vel
            robot_vel.angular.z = ang_vel
        elif key == 's': # Go back
            robot_vel.linear.x = -lin_vel
            robot_vel.angular.z = 0
        elif key == 'e': # Curve right
            robot_vel.linear.x = lin_vel
            robot_vel.angular.z = -ang_vel
        elif key == 'a': # Turn left
            robot_vel.linear.x = 0
            robot_vel.angular.z = ang_vel
        elif key == 'd': # Turn right
            robot_vel.linear.x = 0
            robot_vel.angular.z = -ang_vel
        elif key == 'r': # Emergency stop
            robot_vel.linear.x = 0
            robot_vel.angular.z = 0
        elif key == 'h': # Enable/Disable driving assistance
            if flag == 0: 
                rospy.set_param('/drive_flag', 1)
                flag = 1
            elif flag == 1:
                rospy.set_param('/drive_flag', 0)
                flag = 0
        elif key == 'f': # Quit
            robot_vel.linear.x = 0
            robot_vel.angular.z = 0 
            pub_vel.publish(robot_vel)
            counter1 = 10
            break
        
        # Set the key flag 
        if key == 'w':
            key_flag = 0
        elif key == 'q':
            key_flag = 1
        elif key == 'e':
            key_flag = 2
        else:
            key_flag = 3
        rospy.set_param('/key_flag', key_flag)
        
        # Update message and publishing
        print("Linear velocity: ", lin_vel)
        print("Angular velocity: ", ang_vel)
        pub_vel.publish(robot_vel)
        counter2 += 1
    
# Show the UI
def interface():
    '''
    Provides an interface to choose among different options, which are: 
    0 - Exit and close the program.
    1 - Insert new coordinates to reach.
    2 - Cancel the current goal.
    3 - Pass to the manual driving interface.
    4 - Enable/Disable the driving assistance.
    '''
    
    global flag 
    global counter1
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    res = '5'
    
    goal_flag = 0;
    rospy.set_param('/goal_flag', goal_flag)
    
    while (res != '0'):
        # Print command list
        if counter1 % 10 == 0:
            print("\nChoose an action:\n"
            "0 - Exit\n"
            "1 - Insert new coordinates to reach\n"
            "2 - Cancel the current goal\n"
            "3 - Manual driving")
        if flag == 0:
            print("4 - Enable driving assistance\n")
        elif flag == 1:
            print("4 - Disable driving assistance\n")
        
        # Take user input
        res = input("Action (type the corresponding number): ")

        # Check input
        if res != '0' and res != '1' and res != '2' and res != '3' and res != '4':
            print("\nERROR: type '0', '1', '2', '3' or '4'.\n")
        
        counter1 += 1
        
        goal_flag = rospy.get_param("/goal_flag")
        
        # Exit
        if res == '0':
            rospy.set_param('/goal_flag', 0)
            client.cancel_goal()
        
        # Publish new goal
        elif res == '1':
            print("\nInsert coordinates to reach:");
            x = input("x: ");
            y = input("y: ");
            
            try:
                x = float(x)
                y = float(y)
            except:
                print("Coordinates not valid, please instert only numbers.")
                continue

            # Set goal
            goal_pos = MoveBaseGoal()
            goal_pos.target_pose.header.frame_id = "map"
            goal_pos.target_pose.pose.orientation.w = 1
            goal_pos.target_pose.pose.position.x = float(x)
            goal_pos.target_pose.pose.position.y = float(y)
            
            # Publish
            client.send_goal(goal_pos)
            rospy.set_param('/goal_flag', 1)
            print("Goal sent.\n")
        
        # Cancel current goal
        elif res == '2':
            if goal_flag == 1:
                rospy.set_param('/goal_flag', 0)
                client.cancel_goal()
                goal_flag = 0
                print("Goal cancelled.\n")
            else: 
                print("There is no goal set.\n")
            
        # Manual driving
        elif res == '3':
            if goal_flag == 1:
                rospy.set_param('/goal_flag', 0)
                client.cancel_goal()
            manualDriving()
        
        # Enable/Disable driving assistance
        elif res == '4':
            if flag == 0:
                rospy.set_param('/drive_flag', 1)
                flag = 1
                print("\nDriving assistance enabled.\n")
            elif flag == 1:
                rospy.set_param('/drive_flag', 0)
                flag = 0
                print("\nDriving assistance disabled.\n")
            
def main():
    '''
    Starts the User Interface (UI) to control the robot.
    '''
    
    print("\nWelcome to the User Interface!\n"
        "Here you can choose between two different "
        "modalities to control your robot: automatic "
        "goal reaching or manual driving, with or without "
        "the driving assistance!")
    
    rospy.init_node("rt2_robot_ui")    
    interface()
    print("\nBye.\n")

if __name__ == '__main__':
    main()