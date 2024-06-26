#Author: Mohammad Anwar Meri(s215713)

#!/usr/bin/env python3
#Import libraries
import socket 
from ev3dev.ev3 import *
from ev3dev2.motor import MediumMotor,LargeMotor,MoveTank, OUTPUT_B,OUTPUT_C 
import time
#Define the ip and port number for UDP communication
UDP_IP = "0.0.0.0"  # EV3 IP address
UDP_PORT = 10002

# Initialize a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Initialize motors 
left_motor = LargeMotor('outA')  # Port 1 
right_motor = LargeMotor('outD') # Port 2
drive  = MoveTank(OUTPUT_A, OUTPUT_D, motor_class=LargeMotor) #Motors for driving


takein =  MoveTank(OUTPUT_B, OUTPUT_C, motor_class=MediumMotor) # Motors for collecting and releasing balls


# Function to eject a ball
def eject_ball(arg1,arg2):
    print('inside function')
    takein.on(left_speed = arg1 , right_speed = -arg2)

# Function to collect a ball
def collect_ball(arg1,arg2):
    takein.on(left_speed = -arg1,right_speed = arg2)

#Function to drive
def move_forward(arg1, arg2):
    drive.on(left_speed=arg1,right_speed=arg2)

#Function to drive backward 
def move_backward(arg1, arg2):
    drive.on(left_speed=-arg1,right_speed=-arg2)

#Function to turn left 
def turn_left(arg1, arg2):
    drive.turn_right(arg1,arg2,brake=True,error_margin=0,sleep_time=0.01)

#Function to turn right
def turn_right(arg1, arg2):
    drive.on(left_speed = arg1,right_speed = arg2)
#function to drive by specific time (seconds)
def drive_seconds(arg1,arg2,arg3):
    drive.on_for_seconds(left_speed = arg1, right_speed = arg2,seconds = arg3)

#Main loop to receive command
while True:
    print('inside loop')
    data, addr = sock.recvfrom(1024) # Data it received from the socket 
    message = data.decode() # Decode the data  
    print("Received message: {}".format(message))
    parts = message.split()
    # Parse the message
    command = int(parts[0])
    arg1 = int(parts[1])
    arg2 = int(parts[2])
    arg3 = int(parts[3])

    #Execute the commands
    if command == 0:
        takein.stop()
        drive.stop()
    elif command == 1:
        move_forward(arg1,arg2)
    elif command == 2:
        move_backward(arg1,arg2)
    elif command == 3:
        turn_right(arg1,arg2)
    elif command == 4:
        turn_left()
    elif command == 5:
        print('inside elif statement')
        eject_ball(arg1 ,arg2)
    elif command == 6:
        collect_ball(arg1,arg2)
    elif command == 7:
        drive_seconds(arg1,arg2,arg3)
