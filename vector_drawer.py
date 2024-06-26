#Author: Hermon Asfaha(s224963).

#Importing libraries
import cv2
import numpy as np
#Importing modules
from distance_calculator import find_closest_ball

#Function for drawing vectors and measuring the angle from the robot's direction
def draw_vectors_and_measure_angle(frame, robot_back_position, robot_front_position, ball_positions):

    #augument:
    #frame:  the frame to draw on
    #robot_back_position:  (x,y) coordinates of the robots back position
    #robot_front_position: (x,y) coordinates of the robots front position
    #ball_positions: a list of (x,y) coordinates of balls position 

    angle_degrees = None  # Initialize the angle
    # Checks if robot_back_position and robot_front_position is defined
    if robot_back_position is not None and robot_front_position is not None:
        closest_ball, min_distance = find_closest_ball(robot_front_position, ball_positions) #Finding the closest ball from the robot_front_position
        
        if closest_ball is not None:
            #Draws a circle on the closest ball positions and labels each coordinate and distance 
            ball_x, ball_y = closest_ball[:2] 
            cv2.circle(frame,  (ball_x, ball_y), 10, (0, 255, 255), -1)
            cv2.putText(frame, f'Closest ball: {(ball_x, ball_y)}, Distance: {min_distance:.2f}', 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Draws vectors
            cv2.line(frame, robot_front_position,  (ball_x, ball_y), (255, 0, 0), 2)
            cv2.line(frame, robot_back_position,  (ball_x, ball_y), (0, 255, 0), 2)
            cv2.line(frame, robot_front_position, robot_back_position, (255, 255, 255), 2)
            
            # Measures and displays angle between vectors
            vector1 = np.array(robot_back_position) - np.array(robot_front_position)
            vector2 = np.array( (ball_x, ball_y)) - np.array(robot_front_position)
            dot_product = np.dot(vector1, vector2)
            norm_product = np.linalg.norm(vector1) * np.linalg.norm(vector2)
            angle = np.arccos(np.clip(dot_product / norm_product, -1.0, 1.0))
            angle_degrees = np.degrees(angle)

            
            # Checks the direction of the angle using the cross product and labels the angle 
            cross_product = np.cross(vector1, vector2)
            if cross_product < 0:
                angle_degrees = 360 - angle_degrees
            
            cv2.putText(frame, f'Angle: {angle_degrees:.2f} degrees', 
                        (robot_front_position[0] + 10, robot_front_position[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            # Draws the angle arc
            center = tuple(robot_front_position)
            radius = int(min(np.linalg.norm(vector1), np.linalg.norm(vector2)) / 2)
            
            angle_start = np.degrees(np.arctan2(vector1[1], vector1[0]))
            angle_end = angle_start + angle_degrees

            if angle_end > 360: # Makes sure that the angle is between 0 to 360 degrees
                angle_end -= 360
            
            cv2.ellipse(frame, center, (radius, radius), 0, angle_start, angle_end, (255, 255, 0), 2)
    return frame, angle_degrees
