from __future__ import print_function
import time
from sr.robot import *
import math
import time

'''
    Robotics Engineering - Research Track 1 
    Assignment number 1

    by Simone Contorno
'''

'''
    ---------- GLOBAL VARIABLES ----------
                                            '''

a_th = 2 # Threshold for the control of the orientation
d_th = 0.4 # Threshold for the control of the linear distance

# max_speed = 100
straight_on_speed = 75 # Straight on speed
kp = 1 # Gain
margin_error = 1 # Spin precision

'''
Parameters from golden tokens:
    1. max_angle_from_g : if the robot has an angle less than this one and more than
    min_angle_from_g, it turns to remain parallel to the borders.
    2. min_angle_from_g : if the robot has an angle less than this one and the distance
    from the golden token is less than dist_from_g, it turns of turn_from_g
'''
max_angle_from_g = 80
min_angle_from_g = 10       
dist_from_g = 1
turn_from_g = 90

refresh_rate = 0.05 # Decrement this value to obtain more precision with the robot's movements

print_flag = 0 # Manage printed strings

R = Robot() # Instance of the class Robot

'''
    ---------- FUNCTIONS ----------
                                    '''

'''
Function to go on

Args: 
    speed (int): the speed of the wheels
	seconds (int): the time interval
'''
def drive(speed, seconds):
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

'''
Function to turn
    
Args: 
    speed (int): the speed of the wheels
	seconds (int): the time interval
'''
def turn(speed, seconds):
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

'''
Function to find the closest silver-token

Returns:
    dist (float): distance of the closest token (-1 if no token is detected)
    rot_y (float): angle between the robot and the token (-1 if no token is detected)
'''
def find_token_silver():
    dist = 100
    for token in R.see():
        if token.info.marker_type is MARKER_TOKEN_SILVER and token.dist < dist :
            dist = token.dist
	    rot_y = token.rot_y
    if dist == 100:
	    return -1, -1
    else:
   	    return dist, rot_y

'''
Function to find the closest golden-token

Returns:
	dist (float): distance of the closest token (-1 if no token is detected)
	rot_y (float): angle between the robot and the token (-1 if no token is detected)
'''
def find_token_gold():
    dist = 100
    for token in R.see():
        if token.info.marker_type is MARKER_TOKEN_GOLD and token.dist < dist :
            dist = token.dist
	    rot_y = token.rot_y
    if dist == 100:
	    return -1, -1
    else:
   	    return dist, rot_y

'''
Function to turn the robot of grades (arg) value :
    1. Take current orientation
    2. Compute final orientation
    3. Turn until the orientation is equals to the computed final orientation
'''
def turn_P(grades) :
    # 1. Take current orientation
    heading = R.heading * 180 / math.pi
    if -180 <= heading < 0 :
        heading = module(heading)
    elif 0 <= heading <= 180 :
        heading = 360 - heading
    
    # 2. Compute final orientation
    final = heading + module(grades)

    final_flag = 0

    if final > 360 :
        final -= 360
        error = 360 - heading + final
        final_flag = 1
    else : 
        error = final - heading
        error = module(error)
    
    # 3. Turn until the error is more than max error set
    while error > margin_error :
        if grades > 0 :
            turn(-error * kp, refresh_rate)
        else : 
            turn(+error * kp, refresh_rate)
        
        heading = R.heading * 180 / math.pi
        if -180 <= heading < 0 :
            heading = module(heading)
        elif 0 <= heading <= 180 :
            heading = 360 - heading

        if final_flag == 1 and heading > final :
            error = 360 - heading + final
        else :
            error = final - heading
            error = module(error)
        
        if error > 180 :
            error = 360 - error

'''
Function to move behind the silver token when the robot grabs it:
    the robot checks if there is more space on the right or on the left 
    to avoid the possibility of hurting the silver token against a golden 
    token during the spin.
'''
def move_behind() :
    # Check token on the right
    dist_1 = 100
    for token in R.see(): 
        if token.info.marker_type is MARKER_TOKEN_GOLD and token.dist < dist_1 and 80 < token.rot_y < 100 :
            dist_1 = token.dist
        
    # Check token on the left
    dist_2 = 100
    for token in R.see(): 
        if token.info.marker_type is MARKER_TOKEN_GOLD and token.dist < dist_2 and -100 < token.rot_y < -80 :
            dist_2 = token.dist
    
    if dist_1 > dist_2 : # Turn right
        turn_P(-180)
    else : # Turn left
        turn_P(+180)

    R.release()
    turn_P(180)

'''
Function to get the module of rot (arg)
'''
def module(rot) :
    if rot < 0 :
        return rot * -1
    else :
        return rot

'''
Compute and save each race time
'''
tm_flag = 0 # Manages time computation
wr_flag = 0 # Manages file writing
s_tm = 0 # Initial time
race_counter = 0 # Counts the number of completed races

def race_time():    
    global tm_flag
    global wr_flag
    global s_tm
    global race_counter
    
    # Take the the current time
    if tm_flag == 0:
        s_tm = time.time()
        tm_flag = 1
    
    # Take the current robot position
    location = str(R.location)
    x, y = location.split(",")
    x = x.replace("Vec2(", "")
    y = y.replace(")", "")
    #print("x: " + x)
    #print("y: " + y) 
    
    if float(x) > 0:
        wr_flag = 1
        
    # Check if the robot completed a race
    if wr_flag == 1 and -9 <= float(x) <= -7 and -5 <= float(y) <= -3:
        tm_flag = 0 
        wr_flag = 0
        race_counter = race_counter + 1
        print("Race " + str(race_counter) + " completed")
        
        # Compute the elapsed time
        e_tm = time.time()
        tm = e_tm - s_tm
        sec, dec = str(tm).split(".")
        print("Time elapsed: " + str(tm) + "\n")
        
        # Write the rime value in a file
        f = open("my_time.txt", 'a')
        f.write(sec + "." + dec[0:2] + "\n")
         
'''
    ---------- PROGRAM STARTS ----------
                                        '''

print("\nTurning on the robot...")
time.sleep(3)
print("GO!\n")

while 1:
    # Compute and save each race time
    race_time()
    
    # Search the closest silver token
    dist_s, rot_s = find_token_silver()

    # Search the closest golden token
    dist_g, rot_g = find_token_gold()

    # Take the absolute value of rot_s
    rot_s_m = module(rot_s)
    
    # Take the absolute value of rot_g
    rot_g_m = module(rot_g)

    # Check error
    if dist_s == -1:
        print("I don't see any silver token... :(")
    else :
        # Check if the robot is close enough to grab the silver token
        if dist_s < d_th and rot_s_m < a_th : 
            R.grab() 
            print("Gotcha!\n") 
            move_behind() # Move the token behind 
            print_flag = 1

    '''
    Check if the robot has not any golden token between it and the silver token; 
    in this case it turns against the silver token iff the angle with this one
    is not too much, otherwise it go on to reach it.
    '''
    if (dist_s < dist_g and rot_s_m < 90) :
        if print_flag == 0 : 
            print("Silver token found!")
            print_flag = 2
        elif print_flag == 1 :
            print_flag = 0

        # Go on
        if rot_s_m < a_th : 
            drive(+dist_s * straight_on_speed, refresh_rate)
     
        # Align with the silver token
        else : 
            if rot_s < 0 : # Turn left
                turn(-rot_s_m, refresh_rate)
            else : # Turn right
                turn(+rot_s_m, refresh_rate)

        continue
  
    '''
    Check if the robot is oriented too close to a golden token (the angle between them is too small)
    '''
    if min_angle_from_g <= rot_g_m < max_angle_from_g and dist_g < 1 :
        error = 90 - rot_g_m

        if rot_g < 0 : # Turn right
            turn(+error, refresh_rate)   
        else : # Turn left
            turn(-error, refresh_rate) 

        continue

    '''
    Check if the robot is too close to a golden token (almost in front of it) and turn it towards the free direction
    '''     
    # Check the golden token in front of the robot
    dist_0 = 100
    for token in R.see() : 
        if token.info.marker_type is MARKER_TOKEN_GOLD and token.dist < dist_0 and -min_angle_from_g < token.rot_y < min_angle_from_g :
            dist_0 = token.dist
        
    if dist_0 < 1 : # Change this value depending the width of the path 
        # Check token on the right
        dist_1 = 100
        for token in R.see() : 
            if token.info.marker_type is MARKER_TOKEN_GOLD and token.dist < dist_1 and 80 < token.rot_y < 100 :
                dist_1 = token.dist
        
        if dist_1 == 100 :
            print("Error! I did not find any golden token on the right...")

        # Check token on the left
        dist_2 = 100
        for token in R.see() : 
            if token.info.marker_type is MARKER_TOKEN_GOLD and token.dist < dist_2 and -100 < token.rot_y < -80 :
                dist_2 = token.dist

        if dist_2 == 100 :
            print("Error! I did not find any golden token on the left...")

        # print("Right distance: " + str(dist_1))
        # print("Left distance: " + str(dist_2) + "\n")
        if dist_1 > dist_2 : # Turn right
            print("Turn right...")
        else : # Turn left
            print("Turn left...")

        while dist_0 < 3 : # Change this value depending the width of the path
            if dist_1 > dist_2 : # Turn right
                turn(+straight_on_speed / dist_0, refresh_rate)
            else : # Turn left
                turn(-straight_on_speed / dist_0, refresh_rate)
            
            NULL, rot_g = find_token_gold()
            rot_g_m = module(rot_g)

            for token in R.see() : 
                if token.info.marker_type is MARKER_TOKEN_GOLD and token.dist > dist_0 and -min_angle_from_g < token.rot_y < min_angle_from_g :
                    dist_0 = token.dist

    drive(+straight_on_speed, refresh_rate)
