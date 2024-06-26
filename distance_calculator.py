#Author: Hermon Asfaha(s224963), Mohammad Anwar Meri(s215713)

#Importing libraries
import numpy as np
import math

def calculate_robot_angle(robot_front, robot_back):
#Function to calculcates the angle to the robots front and back position (
    #Argument:
    #robot_front:(x,y) Coordinates of the robots front
    #robot_back: (x,y) Coordinates of the robots back
    delta_x = robot_front[0] - robot_back[0]
    delta_y = robot_front[1] - robot_back[1]
    angle = math.degrees(math.atan2(delta_y, delta_x)) #Calculcates the angle of the coordinates
    return angle #Return the angle

#Function for caculate distance and angle for 
def calculate_distance_and_angle(current_position, target_position):
    #Argument:
    #target_position: (x,y) Target coordinates
    #current_position: (x,y) Starting coordinates
    
    delta_x = target_position[0] - current_position[0]
    delta_y = target_position[1] - current_position[1]
    distance = math.sqrt(delta_x**2 + delta_y**2) #Calculcates the distance of current_position to target_position 
    angle = math.degrees(math.atan2(delta_y, delta_x))  #Calculcates the angle of the coordinates
    return distance, angle # Return the distance and angle 


# Function for calculate distance from 2 points
def calculate_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2) #Return the distance of point1 to point2

#Function for finding the closest ball
def find_closest_ball(robot_front_position, ball_positions):
    #Arguments:
    # robot_front_position: (x,y) coordinates of the robots front position
    # ball_positions: A list of (x,y) coordinates of each ball

    if not ball_positions: # Check if there is no ball_positions 
        return None, float('inf') #Return inf 
    
    closest_ball = None
    min_distance = float('inf')
    
    # For loop for checking all ball_positions 
    for ball in ball_positions:
        distance = calculate_distance(robot_front_position, ball) #Calculcates the distance of robot_front_position to the ball
        if distance < min_distance: #Check if the distance is smaller then min_distance
            min_distance = distance # If true make the distance the new min_distance
            closest_ball = ball # Update the closest ball
            
    return closest_ball, min_distance # Return the closest ball and its distance 

#Function for finding the free kick
def find_closest_free_kick(robot_front_position, free_kick_positions):
     #Arguments:
    # robot_front_position: (x,y) coordinates of the robots front position
    # positions: a list of (x,y) coordinates of positions

    #Check if there is no free_kick_positions or robot_front_position
    if not free_kick_positions or robot_front_position is None:
        return None, float('inf')
    
    closest_free_kick = None
    min_distance = float('inf')
    
    for free_kick in free_kick_positions:
        if free_kick is not None:
            distance = calculate_distance(robot_front_position, free_kick)
            if distance < min_distance:
                min_distance = distance
                closest_free_kick = free_kick
            
    return closest_free_kick, min_distance


#Function for finding the position
def find_closest_position(robot_front_position, positions):
    #Arguments:
    # robot_front_position: (x,y) coordinates of the robots front position
    # positions: a list of (x,y) coordinates of positions
    if not positions or robot_front_position is None:
        return None, float('inf')
    
    closest_position = None
    min_distance = float('inf')
    
    for position in positions:
        if position is not None:
            distance = calculate_distance(robot_front_position, position)
            if distance < min_distance:
                min_distance = distance
                closest_position = position
            
    return closest_position, min_distance    
