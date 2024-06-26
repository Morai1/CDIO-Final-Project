#Author:  Davide Baudino(s224956), Hossein Haidari (s193401) 

#Importing modules
import driving_handler 
#Function for handling turning based of angle  
def turn_robot_func(angle):
    
#Augument:
    #angle: The current angle
        
        if angle > 240 and angle < 360: # Check if angle is between 240 to 360
            driving_handler.turn_right() # Function for for turning right
        elif angle < 240 and angle >= 182 : # Check if angle is between 182 to 240
            driving_handler.turn_right_near_ball() # Function for for light turning right
        elif angle < 120 and angle > 0: # Check if angle is between 0 to 120
            driving_handler.turn_left() # Function for for turning left
        elif angle > 120 and angle <= 178: # Check if angle is between 120 to 178
            driving_handler.turn_left_near_ball() # Function for for light turning left
        else:  
            driving_handler.drive_stop() # Function for stop driving
