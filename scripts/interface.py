# Robotics control in a simulated environment
# Robotics Engineering
# 
# @file ui.py
# @author Simone Contorno (@simone-contorno)
# @copyright Copyright (c) 2022

# ROS headers
from typing import Counter
import rospy
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Manual driving
def manualDriving(flag):
    # Publisher declaration
    pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1000)
    robot_vel = Twist() 
    
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

    return flag, counter1
    
# Show the UI
def interface():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    res = '5'
    
    # Manage printing
    counter1 = 10 
    flag = 0 
    
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
        
        # Exit
        if res == '0':
            rospy.set_param('/time_flag', 0)
            client.cancel_goal()
        
        # Publish new goal
        elif res == '1':
            print("\nInsert coordinates to reach:");
            x = float(input("X: "));
            y = float(input("Y: "));
            
            # Set goal
            goal_pos = MoveBaseGoal()
            goal_pos.target_pose.header.frame_id = "map"
            goal_pos.target_pose.pose.orientation.w = 1
            goal_pos.target_pose.pose.position.x = x
            goal_pos.target_pose.pose.position.y = y
            
            # Publish
            client.send_goal(goal_pos)
            print("Goal sent.\n")
            
        # Cancel current goal
        elif res == '2':
            client.cancel_goal()
            print("Goal cancelled.\n")
            
        # Manual driving
        elif res == '3':
            client.cancel_goal()
            (flag, counter1) = manualDriving(flag)
        
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
    print("User Interface")
    rospy.init_node("jupyter_final_robot")    
    interface()

if __name__ == '__main__':
    main()