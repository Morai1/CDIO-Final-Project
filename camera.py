#Author: Davide Baudino(s224956), Hermon Asfaha(s224963), Mustafa Keskin(s224955). 

#Importing libraries
from ultralytics import YOLO
import cv2
import numpy as np
from math import inf, isinf
import time

#Importing modules
from distance_calculator import find_closest_ball,  find_closest_free_kick
from vector_drawer import draw_vectors_and_measure_angle
import driving_handler  
import grid_drawer  
import turning_function



# Class with all robot state 
class RobotState:
  
    IDLE = 0 # starting state
    SAFEPOINT = 1 # state to drive the to the safepoint 
    CLOSE_BALL_FIND = 2 # state to collect balls 
    RETURN_SAFEPOINT = 3 # state to drive back to the safepoint 
    NEXT_SAFEPOINT = 4 # state to change to the next safepoint
    TARGET_SAFEPOINT = 5 # state to drive from safepoint to the target safepoint next to the goal
    SETUP_GOAL = 6 # state drive to the setup point 
    FREE_KICK = 7 # state to drive to goal point and prepare for score
    SCORE = 8 # state to align and score at goal 
    

# Initialize state variables and other variables
robot_state = RobotState.IDLE
current_angle = None
current_distance = None
kicking_point = True
realigned = False
setup = False
all_ball_positions = []
robot_front_position = None
robot_back_position = None
setup_points = [] 
traversal_points = []
wall_points = []
robot_quadrant = 0
safepoint_reached = False
ball_condition = None
current_ball_count = 0
previous_ball_count = 0
ball_counter = 1
wall_close_front = False
wall_close_back = False
wall_ball_checking = False

# function to handling robot states.
def update_state(angle_degrees, distance_to_ball, all_ball_position ,balls_in_other_quadrants): 
    
# Args:
    # angle_degrees : get the angle of the robot.
    # distance_to_ball : the distance from the robot to the ball/safepoint.
    # all_ball_position: lists the locations to all nearby balls in current quadrant.
    # balls_in_other_quadrants: lists all the other balls in the other quadrants.

# Declare the global variables.
    global robot_state, current_angle, current_distance , kicking_point, realigned,setup, robot_quadrant, ball_condition,current_ball_count,previous_ball_count,ball_counter, wall_ball_checking  
    #wall_close_front = close_to_wall(robot_front_position)
    #wall_close_back = close_to_wall(robot_back_position)
    print("Robot-front is close to wall ",wall_close_front)
    print("Robot-back is close to wall",wall_close_back)
    
    print('all balls positions = ', all_ball_positions)
    current_ball_count = len(all_ball_positions)
    print('current_ball_count = ', current_ball_count)

    # checks the amount of balls collected.
    if current_ball_count < previous_ball_count:
        ball_counter += 1
    if current_ball_count > previous_ball_count:
        ball_counter -= 1
    previous_ball_count = current_ball_count
    
    print('ball counter : ', ball_counter)
    print('previous_ball_count = ', previous_ball_count)
    
    # Checks if robots is in idle state.
    if robot_state == RobotState.IDLE:
        # if no balls detected it will stop all functions and remain in idle.
        if not all_ball_positions:
            driving_handler.stop_all_function()
            robot_state == RobotState
        # if balls detected it continues to the next state.
        else:    
            driving_handler.collector()
            robot_state = RobotState.SAFEPOINT

# drives to safepoint in the current quadrant.
    elif robot_state == RobotState.SAFEPOINT:

        closest_ball_position = all_ball_position[0] # gets the position of the closest safepoint.
        if not all_ball_position:
            print("No safepoint in the same quadrant.")
        print("angle degrees in safepoint",angle_degrees)

        # checks if the robot has reached the safepoint.
        if all_ball_position is not None and robot_front_position is not None and \
            (closest_ball_position[0] - 10 < robot_front_position[0] < closest_ball_position[0] + 10) and \
            (closest_ball_position[1] - 10 < robot_front_position[1] < closest_ball_position[1] + 10): 
            print("we are close to the safepoint")
            driving_handler.drive_stop()
            print("robotstate before changing in safepoint",robot_state)
            robot_state = RobotState.CLOSE_BALL_FIND 
            print("robotstate after changing in safepoint",robot_state)
        
        
       # aligns the robots angle to the safepoint if it.
        elif angle_degrees is not None and (angle_degrees < 178 or angle_degrees > 182):
            current_angle = angle_degrees
            turning_function.turn_robot_func(current_angle)
            print("aligning towards the safepoint")
        else:
            driving_handler.drive_straight()
            print("driving to the safepoint" )

   # collects all the balls in the current quadrant.
    elif robot_state == RobotState.CLOSE_BALL_FIND:
        
        print("Distance inside close_ball_find: ",distance_to_ball)
        print("Is ball infinite ",isinf(distance_to_ball))

        # checks if no balls left in the current quadrant. Changes state if no balls in the current quadrant.
        if not all_ball_position:
            robot_state = RobotState.RETURN_SAFEPOINT
        elif angle_degrees is not None and (angle_degrees < 175 or angle_degrees > 185):
            current_angle = angle_degrees
            turning_function.turn_robot_func(current_angle)
            print("Turning to align with the ball")
        elif distance_to_ball is None or isinf(distance_to_ball):
            driving_handler.drive_stop()
            print("Caught the ball")    
        elif distance_to_ball is not None and distance_to_ball > 10:
            driving_handler.drive_straight()
            print("Driving towards the ball")
            
    
# returns to current safepoint.
    elif robot_state == RobotState.RETURN_SAFEPOINT:
        if not all_ball_position:
            print("No safe point in the same quadrant.")
            return
        print("angle degrees in Return-safepoint",angle_degrees)
        closest_ball_position = all_ball_position[0]
        if (closest_ball_position[0] - 10 < robot_front_position[0] < closest_ball_position[0] + 10) and \
            (closest_ball_position[1] - 10 < robot_front_position[1] < closest_ball_position[1] + 10):
            print("We are close to the return-safepoint")
            driving_handler.drive_stop()
            print("Robot state before changing in return-safepoint", robot_state)
            #changes to next safepoint if other quadrants are not empty.
            if balls_in_other_quadrants:
                robot_state = RobotState.NEXT_SAFEPOINT
            else:
                robot_state = RobotState.TARGET_SAFEPOINT
            print("Robot state after changing in return-safepoint", robot_state)
        
        elif angle_degrees is not None and (angle_degrees < 178 or angle_degrees > 182):
            current_angle = angle_degrees
            turning_function.turn_robot_func(current_angle)
            print("Aligning towards the return-safepoint")
        
        else:
            driving_handler.drive_straight()
            print("Driving to the return-safepoint")
            
    # checks if theres no balls in the next quadrant.
    elif robot_state == RobotState.NEXT_SAFEPOINT:
        if not all_ball_position:
            print("No safepoint in the next quadrant.")
            return
        print("angle degrees in next-safepoint",angle_degrees)
        closest_ball_position = all_ball_position[0]
# checks if it has reached the next safepoint.
        if (closest_ball_position[0] - 10 < robot_front_position[0] < closest_ball_position[0] + 10) and \
           (closest_ball_position[1] - 10 < robot_front_position[1] < closest_ball_position[1] + 10):
            print("We are close to the next safepoint")
            driving_handler.drive_stop()
            print("Robot state before changing in next safepoint", robot_state)
            #checks if no balls left or if it has 6 balls. if so it changes state to target safepoint.
            if not balls_in_other_quadrants or ball_counter == 6:
                robot_state = RobotState.TARGET_SAFEPOINT
            else:
                robot_state = RobotState.SAFEPOINT
           
        elif angle_degrees is not None and (angle_degrees < 178 or angle_degrees > 182):
            current_angle = angle_degrees
            turning_function.turn_robot_func(current_angle)
            print("Aligning towards the next safepoint")
        
        
        else:
            driving_handler.drive_straight()
            print("Driving to the next safepoint")
    # drives to the setup goalpoint.
    elif robot_state == RobotState.SETUP_GOAL:
        
        closest_ball_position = all_ball_position[0]
        
        if not all_ball_position:
            print("No point detected.")
            return
        if all_ball_position is not None and robot_front_position is not None and \
            (closest_ball_position[0] - 10 < robot_front_position[0] < closest_ball_position[0] + 10) and \
            (closest_ball_position[1] - 10 < robot_front_position[1] < closest_ball_position[1] + 10): 
            print("we are close to the setup")
            driving_handler.drive_stop()
            robot_state = RobotState.FREE_KICK
        
        elif angle_degrees is not None and (angle_degrees < 178 or angle_degrees > 182):
            current_angle = angle_degrees
            turning_function.turn_robot_func(current_angle)
            print("aligning towards the setup")
        
        
        else:
            driving_handler.drive_straight()
            print("driving to the setup" )
    #drives to the free kick safepoint.
    elif robot_state == RobotState.FREE_KICK:
        closest_ball_position = all_ball_position[0]
        if not all_ball_position:
            print("No free kick detected.")
            return
        if all_ball_position is not None and robot_front_position is not None and \
            (closest_ball_position[0] - 10 < robot_front_position[0] < closest_ball_position[0] + 10) and \
            (closest_ball_position[1] - 10 < robot_front_position[1] < closest_ball_position[1] + 10): 
            print("we are close to the free kick")
            driving_handler.drive_stop()
            robot_state = RobotState.SCORE
            
            
        elif angle_degrees is not None and (angle_degrees < 178 or angle_degrees > 182):
            current_angle = angle_degrees
            turning_function.turn_robot_func(current_angle)
            print("aligning towards the free kick")
        elif distance_to_ball > 30:
            driving_handler.drive_straight()
            print("driving to the free kick" )
        else:
            driving_handler.drive_straight_near_ball()
            print("near the goal drive slow")
    #aligns itself to the goal and scores.
    elif robot_state == RobotState.SCORE:
        #checks if its aligned to the goal.
        if angle_degrees is not None and (angle_degrees < 179 or angle_degrees > 181):
            current_angle = angle_degrees
            turning_function.turn_robot_func(current_angle)
            print("aligning towards the free kick")
        #it stops and dispenses all the balls into the goal. It backs off from the goal and returns into idle state.
        else: 
            print("Scoring")
            driving_handler.drive_stop() # function for stop drive function
            driving_handler.dispenser() # function for releasing the balls 
           
            driving_handler.drive_seconds(-10,-10,4) # function for driving backwards for 4 seconds 
            time.sleep(2)
            ball_counter = 0 # reset the ball counter 
            robot_state = RobotState.IDLE # return back to idle state 
 
# Function to calculate the traversal points of the overall field.
def calculate_traversal_points(wall_points, offset=135):
# arguments:
# Wall point: the coordinate points in x and y defining the outer boundary.
# offset: the distance to inwardly move from each wall point.

    # Calculates the centroid of each wall point to find the central point to move towards.
    cx = np.mean([p[0] for p in wall_points]) # avg. x coordinate of wall point.
    cy = np.mean([p[1] for p in wall_points]) # avg. y coordinate of wall point.
    
    traversal_points = [] 
    for point in wall_points:
        x, y = point #extracts coordinates (x,y) of the current wall point.

        # Calculates the direction vector to the centroid from the wall point.
        dx = cx - x
        dy = cy - y

        # Normalizes the direction vector.
        dist = np.sqrt(dx**2 + dy**2)
        dx /= dist
        dy /= dist

        # Moves the point inward by the offset and adds the new point to the list.
        traversal_points.append((int(x + dx * offset), int(y + dy * offset)))
    return traversal_points

# Function to identify the quadrant.
def check_quadrant(x, y):
    width=640
    height=480
    mid_x = width // 2
    mid_y = height // 2
#checks the point of each quadrant coordinate.
    if x < mid_x and y < mid_y:
        return 1
    elif x >= mid_x and y < mid_y:
        return 2
    elif x >= mid_x and y >= mid_y:
        return 3
    elif x < mid_x and y >= mid_y:  
        return 4

#defines the types of balls it can collect.
def quadrant_balls(ball_positions):
    balls_in_same_quadrant = []
    balls_in_other_quadrant = []
    for ball in ball_positions: # loop for checking each ball
        ball_x, ball_y, ball_state = ball
        ball_quadrant = check_quadrant(ball_x, ball_y)
        #check if the ball is in samme quadrant as the robot and the ball is not next to the corners and the walls
        if ball_quadrant == robot_quadrant and not (ball_state == 0 or ball_state == 1):  
            balls_in_same_quadrant.append(ball)
            print("ball is in same quadrant as robot")
            #check if the ball is not in samme quadrant as the robot and the ball is not next to the corners and the walls
        elif ball_quadrant != robot_quadrant and not (ball_state == 0 or ball_state == 1):
            balls_in_other_quadrant.append(ball)
    return balls_in_same_quadrant, balls_in_other_quadrant

# checks the type of balls.
def ball_check(all_positions_balls):
    #argument:
    #all_positions_balls: list of coordinates of each ball position on the field.
    top_left_corner = wall_points[0]
    top_right_corner = wall_points[1]
    bottom_left_corner = wall_points[3]
    bottom_right_corner = wall_points[2]
    
    updated_positions_balls = []

    #checks all balls in the list of ball positions.
    for ball in all_positions_balls:
        # extracts coordinates (x,y) and initial state of ball.
        ball_x = ball[0]
        ball_y = ball[1]
        ball_state = ball[2]

        #checks if the balls are located in a corner.
        if(ball_x < top_left_corner[0] + 30 and ball_y < top_left_corner[1] + 30)or \
            (ball_x > top_right_corner[0] - 30 and ball_y < top_right_corner[1] + 30) or \
            (ball_x < bottom_left_corner[0] + 30 and ball_y >  bottom_left_corner[1] - 30  ) or \
            (ball_x > bottom_right_corner[0] - 30 and ball_y > bottom_right_corner[1] - 30):
            ball_state = 0
        
        #checks if the balls are close to the walls.
        elif(ball_x < top_left_corner[0] + 30 and ball_y > top_left_corner[1] +30 and ball_y < bottom_left_corner[1] -30) or \
            (ball_x > top_left_corner[0] + 30 and ball_x < top_right_corner[0] - 30 and ball_y < top_left_corner[1] + 30) or \
            (ball_x > top_right_corner[0] - 30 and ball_y > top_right_corner[1] + 30 and ball_y < bottom_right_corner[1] - 30) or \
            (ball_x < bottom_right_corner[0] - 30 and ball_x > bottom_left_corner[0] + 30 and ball_y > bottom_left_corner[1] - 40):
            ball_state = 1

        #the ball is in the open.
        else:
            ball_state = 2
       #creates a tuple for the updated ball positions and states and adds the updated information into the updated_position_balls list.
        updated_ball = (ball_x, ball_y, ball_state)
        updated_positions_balls.append(updated_ball)
    return updated_positions_balls    

# Function to handle mouse clicks.
def click_event(event, x, y, flags, param):
    #argument:
    #(x,y): the coordinates of the mouse event.
    #event: type of mouse interaction (ex. lbuttondown).
    #flags: additional info about event.
    #param: additional parameters passed to callback.

    global wall_points, traversal_points
    
    #checks if button is pressed and theres less than 4 wall point defined.
    if event == cv2.EVENT_LBUTTONDOWN and len(wall_points) < 4:
        print(f"Clicked at ({x}, {y})")  # Debugging output.
        quadrant = check_quadrant(x, y)
        print(f"Point ({x}, {y}) is in quadrant {quadrant}")  # Debugging output.
        wall_points.append((x, y))
        if len(wall_points) == 4:
            traversal_points.extend(calculate_traversal_points(wall_points))
            print("Traversal points calculated:", traversal_points)  # Debugging output.
            setup_points.extend(traversal_points)  # Add traversal points to setup points.

#function for checking if a point is indside a bounded box
def inside_box(point, box):
    #argument
    #point : the (x,y) coordinate of the point 
    #box: the (x,y) coordinates of the box for points 
    x, y = point
    x_min, y_min, x_max, y_max = box
    return x_min <= x <= x_max and y_min <= y <= y_max



#Function to detect through video feed.
def run_yolo_live():
# Declare the global variables.
    global robot_state, current_angle, current_distance, setup_points, wall_points, traversal_points, all_ball_positions
    
    model = YOLO("final_v17_openvino_model")  # Ensure model name and file are correct.
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Webcam source index is usually 0. 1 is additional camera.
    conf_threshold = 0.4
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
    start_program = False
    class_names = { #defines names for the detected objects.
        0: 'big goal',
        1: 'cross',
        2: 'egg',
        3: 'field',
        4: 'orange ball',
        5: 'robot',
        6: 'robot-back',
        7: 'robot-front',
        8: 'small goal',
        9: 'white ball'
    }
    # defines the colors to each object.
    colors = {
        0: (255, 0, 0),   
        1: (255, 0, 255),  
        2: (255, 255, 0), 
        3: (0, 255, 0),    
        4: (255, 255, 255),  
        5: (255, 255, 255),
        6: (128, 128, 128),
        7: (0, 0, 255),    
        8: (0, 165, 255),
        9: (27, 89, 210)
    }
   #defines variables in the function.
    next_quandrant = True 
    free_kick = None  # Variable to store the coordinate.
    small_goal_midpoint = None
    small_goal_midpoint_setup = None
    free_kick_positions = []
    check = False # Initialize free_kick_positions as a list.
#  def click_event(event, x, y, flags, param):
#     if event == cv2.EVENT_LBUTTONDOWN:
#        points.append((x, y))

# Defines names of the windows and defines mouse click functions to the live feed.
    cv2.namedWindow("YOLO Live Feed")
    cv2.setMouseCallback("YOLO Live Feed", click_event, [wall_points, traversal_points])
    cv2.namedWindow('Live Coordinates of Balls')
    ball_state = 0
    driving_handler.collector()
    
   
    try:
        #main loop to process video frames from the live feed.
        while True: 
            
            ret, frame = cap.read()
            if not ret:
                break
            frame = cv2.resize(frame,(640, 480))
            coordinates_frame = np.zeros((480,640,3), dtype=np.uint8)  # Creates a blank frame for coordinates display.
            
            # perfoms an object detection on the frame.
            results = model(frame, imgsz=640, conf=conf_threshold,verbose = False)
            orange_ball_positions = []
            white_ball_positions = []
            traversal_specific_point = None
            robot_position = None
            wall_ball_in_quadrant = []
            setup_wall_ball_in_quadrant = []

            #access the global variables of the robot'sfront and back and coordinates in the quadrant.
            global robot_front_position
            global robot_back_position
            global robot_quadrant
            crossbox = []
            #Processes the result if the 4 wall points have been defined"
            if len(wall_points) == 4:
                for result in results:
                    #extracts bounding boxes and classes from the results.
                    boxes = result.boxes
                    xyxys = boxes.xyxy.cpu().numpy().astype(int)
                    confs = boxes.conf.cpu().numpy()
                    classes = boxes.cls.cpu().numpy().astype(int)
                    
                    #processes each detection in the result.
                    for xyxy, conf, cls in zip(xyxys, confs, classes):
                        if cls == 1:
                           #if the detected object is in class '1'  it represents an important object to identify distincly.
                            #appends the bounding box coordinates of the specific detection to the crossbox list.
                            crossbox.append((xyxy[0], xyxy[1], xyxy[2], xyxy[3]))
                                            
                    for xyxy, conf, cls in zip(xyxys, confs, classes):
                        if cls == 3:
                            continue
                        #calculates each object's center for positioning.
                        object_center = ((xyxy[0] + xyxy[2]) // 2, (xyxy[1] + xyxy[3]) // 2)
                        cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), colors.get(cls, (0, 255, 0)), 2)
                        if cls in class_names:
                            center_x = (xyxy[0] + xyxy[2]) // 2
                            center_y = (xyxy[1] + xyxy[3]) // 2

                            #checks if class is 9 (white ball).
                            if cls == 9:
                                #checks if the ball's coordinates is in the bounding box.
                                inside_cross = any(inside_box(object_center, box) for box in crossbox)
                                if inside_cross: # if true continue and do not add the ball to the list
                                    continue
                             #adds all the detected white balls into a list.
                                white_ball_positions.append((center_x, center_y,ball_state)) 
                            #checks if class is 4 (orange ball)
                            elif cls == 4:
                                #checks if the ball's coordinates is in the bounding box.
                                inside_cross = any(inside_box(object_center, box) for box in crossbox)
                                if inside_cross:
                                    continue
                             #adds all the detected orange balls into a list.
                                orange_ball_positions.append((center_x, center_y,ball_state))
                            #5,6 and 7 defines the robot's position and the position of its front and back.
                            elif cls == 5:
                                robot_position = (center_x, center_y)
                            elif cls == 6:
                                robot_back_position = (center_x, center_y)
                            elif cls == 7:
                                robot_front_position = (center_x, center_y)
                            elif cls == 8:  # 'small goal'

                                #defines the positions of the goalpoint, midpoint og free kick from small goal.
                                goal_center_x = (xyxy[0] + xyxy[2]) // 2
                                goal_center_y = (xyxy[1] + xyxy[3]) // 2 
                                free_kick = (xyxy[0] - 10, goal_center_y)
                                small_goal_midpoint = (goal_center_x + 50, goal_center_y)
                                small_goal_midpoint_setup =(xyxy[0] - 120, goal_center_y)
                        label = f'{class_names.get(cls, "Unknown class")} ({center_x}, {center_y})'
                        cv2.putText(frame, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        cv2.putText(coordinates_frame, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

             # Draws the small_goal midpoint.
                if small_goal_midpoint:
                    cv2.circle(coordinates_frame, small_goal_midpoint, 5, (0, 255, 255), -1)
                    cv2.putText(coordinates_frame, f"({small_goal_midpoint[0]}, {small_goal_midpoint[1]})", 
                                (small_goal_midpoint[0] + 10, small_goal_midpoint[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Draws the free_kick point to both frames.
                if free_kick:
                    cv2.circle(coordinates_frame, free_kick, 5, (255, 0, 255), -1)
                    cv2.putText(coordinates_frame, f"({free_kick[0]}, {free_kick[1]})", 
                                (free_kick[0] + 10, free_kick[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.circle(frame, free_kick, 5, (255, 0, 255), -1)
                    cv2.putText(frame, f"({free_kick[0]}, {free_kick[1]})", 
                                (free_kick[0] + 10, free_kick[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                # Draws the small_goal_midpoint_setup to both frames.
                if small_goal_midpoint_setup:
                    cv2.circle(coordinates_frame, small_goal_midpoint_setup, 5, (255, 0, 255), -1)
                    cv2.putText(coordinates_frame, f"({small_goal_midpoint_setup[0]}, {free_kick[1]})", 
                                (small_goal_midpoint_setup[0] + 10, small_goal_midpoint_setup[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.circle(frame, free_kick, 5, (255, 0, 255), -1)
                    cv2.putText(frame, f"({free_kick[0]}, {free_kick[1]})", 
                                (small_goal_midpoint_setup[0] + 10, small_goal_midpoint_setup[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
             #Checks which quadrant the robot is located.
                if not check:
                    robot_quadrant = check_quadrant(robot_front_position[0],robot_front_position[1])
                    check = True
                    print(check)

             #Starts program through button.       
                if start_program is True:
                    all_ball_positions = orange_ball_positions + white_ball_positions #saves all the balls positions from the white and orange balls.

                    #the quadrant variables are saved inside each safe point quadrant.
                    safe_point1_quadrant = check_quadrant(*traversal_points[0]) 
                    safe_point2_quadrant = check_quadrant(*traversal_points[1])
                    safe_point3_quadrant = check_quadrant(*traversal_points[2])
                    safe_point4_quadrant = check_quadrant(*traversal_points[3])
                
                # checks and defines all ball types (corner, wall, free ball).
                    all_ball_positions = ball_check(all_ball_positions)
                    
                    print(all_ball_positions)

                    #defines all the ball types it can detect in same and other quadrant.
                    balls_in_same_quadrant, balls_in_other_quadrant = quadrant_balls(all_ball_positions)

                    #saves all ball types it detects in a list.
                    all_ball_positions = balls_in_same_quadrant + balls_in_other_quadrant
                   
                   #checks if the robot is in its idle state.
                    if robot_state == RobotState.IDLE:
                        update_state(0,0,0,0)
                    elif robot_state == RobotState.SAFEPOINT:
                        next_quandrant = True
                        print("robot is trying to get to the safepoint") 

                        #Determines the robots traversal point to the quadrant is in.
                        if safe_point1_quadrant == robot_quadrant: 
                            traversal_specific_point = traversal_points[0] #if the robot is in the first quadrant it will select the first traversal point and save it in the variable.
                        elif safe_point2_quadrant == robot_quadrant: 
                            traversal_specific_point = traversal_points[1]
                        elif safe_point3_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[2]
                        elif safe_point4_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[3]

                        #Creates a list of the specific traversal points.
                        traversal_specific_point_list = [traversal_specific_point] if traversal_specific_point is not None else [] 
                       
                       #Calculates the angle between the robots current and front position and the traversal point.
                        coordinates_frame, traversal_points_angles = draw_vectors_and_measure_angle(coordinates_frame, robot_back_position, robot_front_position, traversal_specific_point_list) 
                       
                       #Finds the distance between the robot to the closest safe point.
                        closest_safe_point, traversal_points_distance = find_closest_free_kick(robot_front_position, traversal_specific_point_list)
                        
                        #Save robot's state by calculating the distance and angles of the points.
                        update_state(traversal_points_angles, traversal_points_distance,traversal_specific_point_list,balls_in_other_quadrant)
                            
                   # checks if the robot is in the close_ball_find state.
                    elif robot_state == RobotState.CLOSE_BALL_FIND:
                        print(balls_in_same_quadrant)

                        #Calculates the angles and vectors of the ball in the same quadrant as the robot.
                        coordinates_frame, quandrant_ball_angle_degrees = draw_vectors_and_measure_angle(coordinates_frame, robot_back_position, robot_front_position, balls_in_same_quadrant) 
                        print("Robot is ball finding")
                       
                       #find the location and distance of the closest ball in the same quadrant.
                        closest_ball, distance_to_quadrant_ball = find_closest_ball(robot_front_position, balls_in_same_quadrant)
                       
                       #Logs the distance and angle to the closest ball.
                        print(quandrant_ball_angle_degrees,balls_in_same_quadrant,distance_to_quadrant_ball)
                      
                       #Saves the state of the robot with new positioning data.
                        update_state(quandrant_ball_angle_degrees, distance_to_quadrant_ball,balls_in_same_quadrant,balls_in_other_quadrant)
                        
                   #checks if the robot state is in return_safepoint.
                    elif robot_state == RobotState.RETURN_SAFEPOINT:
                        
                        #Determines the robots traversal point to the current quadrant. 
                        if safe_point1_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[0]
                        elif safe_point2_quadrant == robot_quadrant: 
                            traversal_specific_point = traversal_points[1]
                        elif safe_point3_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[2]
                        elif safe_point4_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[3]
                        
                        traversal_specific_point_list = [traversal_specific_point] if traversal_specific_point is not None else [] 
                        coordinates_frame, traversal_points_angles = draw_vectors_and_measure_angle(coordinates_frame, robot_back_position, robot_front_position, traversal_specific_point_list) 
                        closest_safe_point, traversal_points_distance = find_closest_free_kick(robot_front_position, traversal_specific_point_list)
                        
                        #saves the state with new data.
                        update_state(traversal_points_angles, traversal_points_distance,traversal_specific_point_list,balls_in_other_quadrant)
                    
                    #checks if the robot state is in next_safepoint.
                    elif robot_state == RobotState.NEXT_SAFEPOINT:
                        print("Moving to next safepoint")

                        #checks if the next_quadrant is true.
                        if next_quandrant is True:
                            robot_quadrant += 1 #add value into the counter.
                            if robot_quadrant > 4: #if the value is above 4 it goes back to 1.
                                robot_quadrant = 1
                            next_quandrant = False
                        
                        print(robot_quadrant)
                        
                        #Determines the robots traversal point to the current quadrant.
                        if safe_point1_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[0]
                        elif safe_point2_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[1]
                        elif safe_point3_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[2]
                        elif safe_point4_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[3]
                        traversal_specific_point_list = [traversal_specific_point] if traversal_specific_point is not None else []
                        coordinates_frame, traversal_points_angles = draw_vectors_and_measure_angle(coordinates_frame, robot_back_position, robot_front_position, traversal_specific_point_list)
                        closest_safe_point, traversal_points_distance = find_closest_free_kick(robot_front_position, traversal_specific_point_list)
                        update_state(traversal_points_angles, traversal_points_distance, traversal_specific_point_list, balls_in_other_quadrant) 
                    

                    #checks if the robot state is in target_safepoint.
                    elif robot_state == RobotState.TARGET_SAFEPOINT:
                        print("robot_state",robot_state,"robot_quadrant", robot_quadrant) 
                        next_quandrant = True  

                        #Determines the robot's next action based on its current quadrant and safepoint.
                        if safe_point1_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[0] #if the robot is in the first traversal point it has to travel to the next safepoint.
                            robot_state = RobotState.NEXT_SAFEPOINT
                            print("We are not in a goal quadrant")
                        elif safe_point2_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[1] #if the robot is in the second traversal point it can travel directly to the setup_goal.
                            robot_state = RobotState.SETUP_GOAL
                            print("We are  in a goal quadrant")
                        elif safe_point3_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[2]
                            robot_state = RobotState.SETUP_GOAL
                            print("We are  in a goal quadrant")
                        elif safe_point4_quadrant == robot_quadrant:
                            traversal_specific_point = traversal_points[3]
                            robot_state = RobotState.NEXT_SAFEPOINT
                            print("We are not in a goal quadrant")
                        
                    #checks if the state is in setup_goal.  
                    elif robot_state == RobotState.SETUP_GOAL:
                        small_goal_setup_positions = [small_goal_midpoint_setup] if small_goal_midpoint_setup is not None else []
                        coordinates_frame, small_goal_setup_angle_degrees = draw_vectors_and_measure_angle(coordinates_frame, robot_back_position, robot_front_position, small_goal_setup_positions) 
                        closest_setup, small_goal_setup_distance = find_closest_free_kick(robot_front_position, small_goal_setup_positions) 
                        
                        print(small_goal_setup_positions)
                        print(small_goal_setup_angle_degrees)
                        print(small_goal_setup_distance)
                        update_state(small_goal_setup_angle_degrees,small_goal_setup_distance,small_goal_setup_positions,balls_in_other_quadrant)
                    elif robot_state == RobotState.FREE_KICK:
                        free_kick_positions = [free_kick] if free_kick is not None else [] 
                        coordinates_frame, free_kick_angle = draw_vectors_and_measure_angle(coordinates_frame, robot_back_position, robot_front_position, free_kick_positions) 
                        closest_small_goal, free_kick_distance = find_closest_free_kick(robot_front_position, free_kick_positions)
                        update_state(free_kick_angle, free_kick_distance,free_kick_positions,balls_in_other_quadrant)
                    elif robot_state == RobotState.SCORE:
                        small_goal_positions = [small_goal_midpoint] if small_goal_midpoint is not None else []                                
                        all_ball_positions = orange_ball_positions + white_ball_positions
                        coordinates_frame, small_goal_angle_degrees = draw_vectors_and_measure_angle(coordinates_frame, robot_back_position, robot_front_position, small_goal_positions) 
                        closest_pen_kick, small_goal__distance = find_closest_free_kick(robot_front_position, small_goal_positions)
                        update_state(small_goal_angle_degrees,small_goal__distance,small_goal_positions,balls_in_other_quadrant)
        
                # Drawing wall points.
                for i, point in enumerate(wall_points):
                    cv2.circle(coordinates_frame, point, 5, (0, 0, 255), -1)
                    cv2.putText(coordinates_frame, f"({point[0]}, {point[1]})", (point[0] + 10, point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                if len(wall_points) == 4:
                    cv2.line(coordinates_frame, wall_points[0], wall_points[1], (255, 0, 0), 2)
                    cv2.line(coordinates_frame, wall_points[1], wall_points[2], (255, 0, 0), 2)
                    cv2.line(coordinates_frame, wall_points[2], wall_points[3], (255, 0, 0), 2)
                    cv2.line(coordinates_frame, wall_points[3], wall_points[0], (255, 0, 0), 2)

                # Drawing traversal points.
                for i, point in enumerate(traversal_points):
                    cv2.circle(coordinates_frame, point, 5, (0, 255, 0), -1)
                    cv2.putText(coordinates_frame, f"({point[0]}, {point[1]})", (point[0] + 10, point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                if len(traversal_points) == 4:
                    cv2.line(coordinates_frame, traversal_points[0], traversal_points[1], (0, 255, 0), 2)
                    cv2.line(coordinates_frame, traversal_points[1], traversal_points[2], (0, 255, 0), 2)
                    cv2.line(coordinates_frame, traversal_points[2], traversal_points[3], (0, 255, 0), 2)
                    cv2.line(coordinates_frame, traversal_points[3], traversal_points[0], (0, 255, 0), 2)

                # Draws the grid on the coordinates frame.
                coordinates_frame = grid_drawer.draw_grid(coordinates_frame)

            #Displays the current frame og the live video and additional window with the coordinated of the detected objects.
            cv2.imshow('YOLO Live Feed', frame)
            cv2.imshow('Live Coordinates of Balls', coordinates_frame)

            #Waits for key press for 1 ms.
            key = cv2.waitKey(1) & 0xFF
            
            #Program starts if the 's' key is pressed.
            if key == ord('s'):
                start_program = True
                
            #Program closes if the 'q' key is pressed.
            if key == ord('q'):
                break
                
    #Makes sure to close all the windows when the loop exits.
    finally:
        cap.release()
        cv2.destroyAllWindows()

# Execute the run_yolo_live() function only if this script is run as the main program.
if __name__ == "__main__":
    run_yolo_live() 