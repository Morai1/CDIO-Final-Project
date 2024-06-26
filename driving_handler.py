#Author: Davide Baudino(s224956), Hossein Haidari(s193401).

#Importing libraries
import socket
import time

UDP_IP = "169.254.219.69" # the Ip adress for connecting to ev3(change ip adress to match with ev3)
UDP_PORT = 10002 # The port number for connecting to ev3 (change the port to match same) 

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

#Function for driving straight
def drive_straight():
    
    type = 1  # Define type of action the robot should do (1 is for driving stright)
    motor1 = 10 # Define the power the motor 1 (0-100)
    motor2 = 10 #Define the power the motor 2 (0-100)
    dummy = 0 # Extra dummy data
    run = f'{type} {motor1} {motor2} {dummy}' # Save f-string of all variables into run
    encoded_run = run.encode() # Encoded run into bytes 
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT)) # Send the encode command to the robot via UDP to the ip and port

# Function for slow straight
def drive_straight_near_ball():
    type = 1 # Define type of action the robot should do (1 is for driving)
    motor1 = 4 # Define the power the motor 1 (0-100)
    motor2 = 4  #Define the power the motor 2 (0-100)
    dummy = 0 # Extra dummy data
    run = f'{type} {motor1} {motor2} {dummy}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))    
# Function for turning left lightly
def turn_left_near_ball():
    type = 1
    motor1 = 0
    motor2 = 4
    dummy = 0
    run = f'{type} {motor1} {motor2} {dummy}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))
# Function for turning left
def turn_left():
    type = 1
    motor1 = 0
    motor2 = 10
    dummy = 0
    run = f'{type} {motor1} {motor2} {dummy}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))
# Function for turning right
def turn_right():
    type = 1
    motor1 = 10
    motor2 = 0
    dummy = 0
    run = f'{type} {motor1} {motor2} {dummy}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))

# Function for turning right lightly 
def turn_right_near_ball():
    type = 1
    motor1 = 4
    motor2 = 0
    dummy = 0
    run = f'{type} {motor1} {motor2} {dummy}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))

# Function for stoping drive function
def drive_stop():
    type = 1
    motor1 = 0
    motor2 = 0
    dummy = 0
    run = f'{type} {motor1} {motor2} {dummy}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))

#Function for collectoring the balls 
def collector():
    type = 6 # Define type of action the robot should do (6 is for collecting balls)
    motor1 = 50
    motor2 = 50
    dummy = 0
    run = f'{type} {motor1} {motor2} {dummy}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))

#Function for releasing the balls 
def dispenser():
    type = 5 # Define type of action the robot should do (5 is for releasing)
    motor1 = 99
    motor2 = 99
    dummy = 0
    run = f'{type} {motor1} {motor2} {dummy}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))

#Functuion for stoping all robot function
def stop_all_function():
    type = 0 # Define type of action the robot should do (0 is for stopping all the robot motors)
    motor1 = 0
    motor2 = 0
    dummy = 0
    run = f'{type} {motor1} {motor2} {dummy}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))

#Functuion for drive backwards 
def drive_backwards():
    type = 2 # Define type of action the robot should do (2 is for driving backwards)
    motor1 = 10
    motor2 = 10
    dummy = 0
    run = f'{type} {motor1} {motor2} {dummy}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))

# Function for driving by define by second
def drive_seconds(x,y,second):
    #Argumment:
    #y: Define the power for motor 1
    #x: Define the power for motor 2
    #second: how long is its should drive for in seconds
    type = 7 # define type of action the robot should do (7 is for driving for specific time in second)
    motor1 = x
    motor2 = y
    seconds = second
    run = f'{type} {motor1} {motor2} {seconds}'
    encoded_run = run.encode()
    sock.sendto(encoded_run, (UDP_IP, UDP_PORT))
    