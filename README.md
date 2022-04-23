# Research Track 2 assignment - Robotics Engineering
## Control of a robot in a simulated environment (Jupyter Notebook)
### Author: Simone Contorno

<br>

### Introduction
An overview of this program.<br>
[Go to Introduction](#intro)

### How it works
A rapid description of how the program works (pseudo-code).<br>
[Go to How it works](#how)

### Installation and Execution
How install and run this program in Linux.<br>
[Go to Installation and Execution](#installation)

### Improvements
How Jupyter Notebook interface could be improved.<br>
[Go to Improvements](#improve)

<a name="intro"></a>
### Introduction

The main aim of this repository is provide an alternative, more modular, implementation of the program that can be downloaded at this link:<br>
https://github.com/simone-contorno/rt-assignment-3<br>

Now, the User Interface, wrote in python, is implemented in a different node and also in Jupyter. <br>

<a name="how"></a>
### How it works

There are 3 files:
<ul>
    <li>rt2_robot_logic.cpp: simulates the 'logic' of the robot.</li>
    <li>rt2_robot_interface.py: provides the UI (User Interface) to the user.</li>
    <li>rt2_robot_jupyter_ui.ipynb: provides the graphic UI (User Interface) to the user.</li>
</ul>

#### rt2_robot_logic.cpp
Managing publishers, subscribers and server parameters it is able to avoid the crashing of the robot against a wall, if asked by the user, and register how many targets it reached and how many not. A target is considered 'not reached' when the robot does not reach it within 2 minutes, then the goal is cancelled.<br>
There are 3 subscribers that run simultaneously thanks to a multi-thread architecture given by the ROS class AsyncSpinner:
<ul>
    <li>sub_pos: subscribes to the topic /move_base/feedback through the function currentStatus that continuously update the current goal ID and check
        whether the robot has reached the goal position.</li>
    <li>sub_goal: subscribes to the topic /move_base/goal through the function currentGoal that continuously update the current goal coordinates.</li>
    <li>sub_laser: subscribes to the topic /scan through the function drivingAssistance that continuously take data by the laser scanner and, if the
        driving assistance is enabled, help the user to drive the robot stopping it if there is a wall too close in the current direction.</li>
</ul>

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

##### Pseudo-code
<pre><code>
FUNCTION drivingAssistance WITH (msg)
    COMPUTE minimum distance on the right
    COMPUTE minimum distance in the middle
    COMPUTE minimum distance on the left
    
    IF driving assistance is enabled AND
        the robot is going against a wall THEN
        SET robot velocity TO 0
        PUBLISH robot velocity
    ENDIF

    IF a goal position is set THEN
        COMPUTE the time elapsed
        IF the time elapsed IS GREATER THAN 120 seconds THEN
            DELETE current goal
            INCRESE number of non-reached goals
        ENDIF
    ENDIF
ENDFUNCTION

FUNCTION currentStatus WITH (msg) 
    SET current robot position
    COMPUTE the difference between the current robot position and the current goal position
    
    IF the difference IS LESS THAN 0.5 THEN
        STOP to compute the elapsed time
        INCREASE number of reached goals
    ENDIF
    
    UPDATE the goal ID if there is a new one
ENDFUNCTION

FUNCTION currentGoal WITH (msg)
    SET current goal position
ENDFUNCTION

FUNCTION main WITH (argc, argv)
    INITIALIZE the node "rt2_robot_logic"
    
    SET the first publisher TO "move_base/cancel"
    SET the second publisher TO "cmd_vel"

    SET the first subscriber TO "/move_base/feedback" WITH currentStatus
    SET the second subscriber TO "/move_base/goal" WITH currentGoal
    SET the third subscriber TO "/scan" WITH drivingAssistance

    INITIALIZE spinner WITH 3 threads
    START spinner
    INPUT "If you want to quit, enter any key:"
    PRINT "Bye."
    STOP spinner
    CALL ros::shutdown
    CALL ros::waitForShutdown

    RETURN 0
ENDFUNCTION
</code></pre>

#### rt2_robot_interface.py
The user will be able to choose between two different modalities:
<ul>
    <li>Automatic goal reaching.</li>
    <li>Manual driving with or without the driving assistance.</li>
</ul>
There is also the possibility to cancel the current goal, if any.

##### Pseudo-code
<pre><code>
FUNCTION manualDriving
    WHILE user does not quit
        TAKE user input through the keyboard
        EXEC corresponding task
        PUBLISH new robot velocity
    ENDWHILE
ENDFUNCTION

FUNCTION userInterface 
    WHILE user does not quit
        TAKE user input through the keyboard
        EXEC corresponding task
    ENDWHILE
ENDFUNCTION

FUNCTION main WITH (argc, argv)
    INITIALIZE the node "rt2_robot_ui"
    PRINT starting message
    CALL interface
    PRINT "Bye."
    
    RETURN 0
ENDFUNCTION
</code></pre>

#### rt2_robot_jupyter_ui.ipynb
Additionally with respect to the 'rt2_robot_interface.py" file, in Jupyter, it is possible to plot:
<ul>
    <li>the robot position (with and without tracking all the history);</li>
    <li>the laser scanner data;</li> 
    <li>the number of reached and non-reached goals.</li>
</ul>
These can be plot using a dedicated cell (Section 4), or directly by the UI (Section 3) (discouraged, read the Improvements section).<br>
In order to use the first modality: uncomment the Section 1.1 and comment the 1.2.<br>
In order to use the second modality: comment the Section 1.1 and uncomment the 1.2.<br>
The first one is used by default.<br>
It is also possible visualize the 3D map of Rviz running the Section 5.<br>
<br>
To read the documentation of the files "rt2_robot_interface.py" and "rt2_robot_logic.cpp" go to this link:<br>
https://simone-contorno.github.io/rt2-assignment/files.html.

<a name="installation"></a>
### Installation and Execution
Go into the src folder of your ROS workspace.<br> 
Download this repository:

<pre><code>git clone https://github.com/simone-contorno/rt2-assignment</code></pre>

Go into the root folder of your ROS workspace and build the workspace: 

<pre><code>catkin_make</code></pre>

Afterwards refresh the package list:

<pre><code>rospack profile</code></pre>

Now, to launch all these nodes using the launch file final_assignment.launch, install xterm: 

<pre><code>sudo apt install xterm</code></pre>

Launch the launch file:

<pre><code>roslaunch rt2_robot rt2_robot.launch</code></pre>

Alternatively, if you don't want intall xterm, you can open 4 terminals and:
<ul> 
    <li>In the first one launch the environment:
        <pre><code>roslaunch rt2_robot simulation_gmapping.launch</code></pre>
    </li>
    <li>In the second one launch the action server move_base:
        <pre><code>roslaunch rt2_robot move_base.launch</code></pre>
    </li>
    <li>In the third one run the logic of the robot:
        <pre><code>rosrun rt2_robot rt2_robot_logic</code></pre>
    </li>
    <li>In the fouth one run the user interface:
        <pre><code>rosrun rt2_robot rt2_robot_interface.py</code></pre>
    </li>
</ul>

In this second case, be careful to run the rt2_robot_logic before starting to use the user interface, otherwise you can encounter some troubles!<br>

If you want to use the Jupyter Notebook interface, you need to start it:

<pre><code>jupyter notebook --allow-root --ip 0.0.0.0</code></pre>

Now you can access to the rt2_robot folder in your ROS workspace and open the file rt2_robot_jupyter_ui.ipynb.<br>
Once open, run all the cells.<br>
By default, you will find:
<ul>
    <li>The User Interface to control the robot in the Section 3.</li>
    <li>The data plots in the Section 4.
        <ul>
            <li>Robot position data: Section 4.3.</li>
            <li>Laser scanner data: Section 4.4.</li>
            <li>Number of reached/non-reached targets: Section 4.5.</li>
        </ul>    
    </li>
    <li>The 3D map in the Section 5.</li>
</ul>
Notice that, using the defaul plot modality, in order to update the number of reached/non-reached targets in the third plot, you need to click on the "Update targets plot" button. <br>
If you change the plot modality in the Section 1, then you will not find the data plots in the Section 4, but you can plot them by the User Interface in the Section 3.

<a name="improve"></a>
### Improvements

In Jupyter Notebook, when data are plotted directly by the UI in the Section 3, the kernel remains busy and it cannot be possible continue to use the interface until the plotting windows will be closed. Then, a good improvement would be allow to the user to continue to use the interface while it is plotting the data. <br>
Also, using the default plot modality, update the number of reached/non-reached targets dinamically, without click on the update button, would be a good user experience improvement.
