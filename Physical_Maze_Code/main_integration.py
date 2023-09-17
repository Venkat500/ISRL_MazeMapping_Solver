#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


import astar

#This is using the move.py to call the movements instead of throing them all into a single program
import move_integration

#This is used to mape the maze 
import mazeMapping_integration
import Direction
import pickle
import socket


# This is Part 1 of Maze Mapping: the Maze Generator algorithm
# through a DSF pre-order approach.
# It will return a fully explored maze.
# Value 0  -> Approachable cells
# Value -2 -> Obstacles
maze, facing_direction = mazeMapping_integration.mazeMap()

#This is Part 2 of Finding the Shortest Path using A*(A star) algorithm
start = (0, 0)
goal = (1, 1)  # This now directly represents the goal location in the maze

#Socket 
bind_ip = "0.0.0.0"
bind_port = 27700

# Set up the server to listen for the processed data
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((bind_ip, bind_port))
sock.listen(1)
print("Server is now listening...")

#This is a maze solving Algorithm Called A*star, uses the functions defined in the astar.py
path = astar.a_star_search(maze, start, goal)
print("Answer :", path)
Direction.calculate_direction(path)
#This Variable indicates the direction in which the robot faces, useful in helping the robot to determine the next move
# facing_direction="up"



# Invoke the client function to process the path data and send it back
path = [(0,0), (0,1), (1,1)]
Direction.calculate_direction(path)

# Now, the server waits for the processed data from the client
connection, client_address = sock.accept()

data = b""
while True:
    packet = connection.recv(4096)
    if not packet: 
        break
    data += packet

received_data = pickle.loads(data)
print(received_data)

#Printting the Maze
print("\n\n =========== Generated Maze ============")
for i in maze:
    print(i)    

facing_direction = "up"
print("=========== Finalized Directions ===============")
print ("Directions : ", received_data)
print("================================================")
for i in received_data:
    print("--------------------------------------------")
    if i=="down":
        print("Moving Down V")
        facing_direction = move_integration.down(facing_direction)
        wait(2000)
    elif i=="up":
        print(" Moving Up ^")
        facing_direction = move_integration.up(facing_direction)
        wait(2000)
    elif i=="left":
        print("Moving Left <=")
        facing_direction = move_integration.left(facing_direction)
        wait(2000)
    elif i=="right":
        print("Moving Right =>")
        facing_direction = move_integration.right(facing_direction) 
        wait(2000)
        