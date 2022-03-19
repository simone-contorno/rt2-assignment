# RT2-Assignment
## Assignment of Research Track 2 - Robotics Engineering
### Author: Simone Contorno

<br>

Control of a robot in a simulated environment using Jupyter

### Introduction
An overview of this program function.<br>
[Go to Introduction](#intro)

### How it works
A rapid description of how the program works (pseudo-code).<br>
[Go to How it works](#how)

### Installation and Execution
How install and run RT2-Assignment in Linux.<br>
[Go to Installation and Execution](#installation)

### Improvements
How this program could be improved.<br>
[Go to Improvements](#improve)

<a name="intro"></a>
### Introduction

This program manage a robot, endowed with laser scanners, which should move autonomously inside a map.<br>
You can use the user interface to:
<ol>
    <li>Let the robot to autonomously reach a x,y coordinate inserted by command line.</li>
    <li>Drive the robot with the keyboard.</li>
    <li>Drive the robot with the keyboard availing of a simple driving assistance.</li>
</ol>

The map is this one:<br>
<br>Rviz:<br>
<img src="https://github.com/simone-contorno/RT2-Assignment/blob/main/third_assignment_map_rviz.png" width="275" height="377">
<br><br>Gazebo:<br>
<img src="https://github.com/simone-contorno/RT2-Assignment/blob/main/third_assignment_map_gazebo.png" width="500" height="259">

<a name="how"></a>
### How it works

The program use the launch file "simulation_gmapping.launch" to run the simulated environment, and the launch file "move_base.launch" to run the action move_base that provides several topics, including:
<ul>
    <li>move_base/goal to publish the goal position;</li>
    <li>move_base/feedback to receive the feedback;</li> 
    <li>move_base/cancel to cancel the current goal.</li>
</ul>

'move_base' can also be used as an Action Server. This program use both the methods; in particular, there are 2 files:
<ul>
    <li>final_robot.cpp : uses 'move_base' topics to receive the feedback and cancel the current goal if this is not reached within a certain amount of time (assuming that the goal point cannot be reached). The main aim of this node is to manage the "logic" of the robot; in particular, there are 3 subscribers that run simultaneously thanks to a multi-thread architecture given by the ROS class AsyncSpinner::
        <ul>
            <li>sub_pos: subscribes to the topic /move_base/feedback through the function currentStatus that continuosly update the current goal ID and check whether the robot has reached the goal position.</li>
            <li>sub_goal: subscribes to the topic /move_base/goal through the function currentGoal that continuosly update the current goal coordinates. </li>
            <li>sub_laser: subscribes to the topic /scan through the function drivingAssistance that continuosly take data by the laser scanner and, if the driving assistance is enabled, help the user to drive the robot stopping it if there is a wall too close in the current direction.</li>
        </ul>
        <br>
        You can change 3 constant values to modify some aspect of the program:
        <ul>
            <li>DIST: minimum distance from the wall with the driving assistance enabled.</li>
            <li>POS_ERROR: position range error.</li>
            <li>MAX_TIME: maximum time to reach a goal (microseconds).</li>
        </ul>
        In the code these appear like:
        ``` cpp
        #define DIST 0.35 
        #define POS_ERROR 0.5 
        #define MAX_TIME 120000000 
        ```
        <br>
        A short description of the program behavior is this one:
        <pre><code>
        FUNCTION drivingAssistance WITH (msg)
            COMPUTE minimum distance on the right
            COMPUTE minimum distance in the middle
            COMPUTE minimum distance on the left
            <br> 
            IF driving assistance is enabled AND
                the robot is going against a wall THEN
                SET robot velocity TO 0
                PUBLISH robot velocity
            ENDIF
            <br>
            IF a goal position is set THEN
                COMPUTE the time elapsed
                IF the time elapsed IS GREATER THAN 120 seconds THEN
                    DELETE current goal
                ENDIF
            ENDIF
        ENDFUNCTION
        <br>
        FUNCTION currentStatus WITH (msg) 
            SET current robot position
            COMPUTE the difference between the current robot position
            and the current goal position
            IF the difference IS LESS THAN 0.5 THEN
                STOP to compute the elapsed time
            ENDIF
        ENDFUNCTION
        <br>
        FUNCTION currentGoal WITH (msg)
            SET current goal position
        ENDFUNCTION
        <br>
        FUNCTION main WITH (argc, argv)
            INITIALIZE the node "final_robot"
            <br>
            SET the second publisher TO "move_base/cancel"
            SET the third publisher TO "cmd_vel"
            <br>
            SET the first subscriber TO "/move_base/feedback" WITH currentStatus
            SET the second subscriber TO "/move_base/goal" WITH currentGoal
            SET the third subscriber TO "/scan" WITH drivingAssistance
            <br>
            INITIALIZE spinner WITH 3 threads
            START spinner
            WAIT for the user 
            STOP spinner
            CALL ros::shutdown
            CALL ros::waitForShutdown
            <br>
            RETURN 0
        ENDFUNCTION
        </code></pre>
    </li>
    <li>interface.py : uses 'move_base' as an Action Server to publish anche cancel the goals. It provide an user interface the allows to the user to manage the robot in order to:
        <ol>
            <li>Autonomously reaching a goal position: 
                <ul>
                    <li>ask to the user to insert the coordinates x and y to reach;</li>
                    <li>save the current time;</li>
                    <li>set the frame_id to "map" (corresponding to the environment that is used) and the new coordinates to reach;</li>
                    <li>send the new goal.</li>
                </ul>
            </li>
            <li>Cancel the current goal.</li>
            <li>Be driven by the user through the keyboard (the list of commands is printed on the console).</li>
        </ol>
        A short description of the program behavior is this one:
        <pre><code>
        FUNCTION manualDriving
            WHILE user does not quit
                TAKE user input through the keyboard
                EXEC corresponding task
                PUBLISH new robot velocity
            ENDWHILE
        ENDFUNCTION
        <br>
        FUNCTION userInterface 
            WHILE user does not quit
                TAKE user input through the keyboard
                EXEC corresponding task
            ENDWHILE
        <br>
        FUNCTION main WITH (argc, argv)
            INITIALIZE the node "interface"
            CALL interface
        ENDFUNCTION
        </code></pre>
    </li>
</ul>

<a name="installation"></a>
### Installation and Execution

Open the terminal, and download this repository:

<pre><code>git clone https://github.com/simone-contorno/RT2-Assignment</code></pre>

Copy or move the folder final_assignment into the src folder of your ROS workspace.<br> 
Go into the root folder of your ROS workspace and type: 

<pre><code>catkin_make</code></pre>

Afterwards type:

<pre><code>rospack profile</code></pre>

Now, to launch all these nodes using the launch file final_assignment.launch, install xterm: 

<pre><code>sudo apt install xterm</code></pre>

Launch the launch file:

<pre><code>roslaunch final_assignment final_assignment.launch</code></pre>

Alternatively, if you don't want intall xterm, you can open 4 terminals and:
<ul> 
    <li>In the first one launch the environment:
        <pre><code>roslaunch final_assignment simulation_gmapping.launch</code></pre>
    </li>
    <li>In the second one launch the action move_base:
        <pre><code>roslaunch final_assignment move_base.launch</code></pre>
    </li>
    <li>In the third one run the node final_robot:
        <pre><code>rosrun final_assignment final_robot</code></pre>
    </li>
    <li>In the fouth one run the node interface:
        <pre><code>rosrun final_assignment interface</code></pre>
    </li>
</ul>

In this second case, be careful to run the final_robot before starting to use the user interface, otherwise you can encounter some troubles!

<a name="improve"></a>
### Improvements

The driving assistance can be improved by moving the robot in the right direction when the user is driving 
it against a wall, instead of just stop it.<br><br>

Thanks to have read this file, i hope it was clear and interesting.
