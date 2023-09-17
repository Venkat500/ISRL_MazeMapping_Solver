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

# This is Part 1 of Maze Mapping: the Maze Generator algorithm
# through a DSF pre-order approach.
# It will return a fully explored maze.
# Value 0  -> Approachable cells
# Value -2 -> Obstacles
maze, facing_direction = mazeMapping_integration.mazeMap()

#This is Part 2 of Finding the Shortest Path using A*(A star) algorithm
start = (0, 0)
goal = (1, 1)  # This now directly represents the goal location in the maze

#This is a maze solving Algorithm Called A*star, uses the functions defined in the astar.py
path = astar.a_star_search(maze, start, goal)
print("Answer :", path)
Direction.calculate_direction(path)
#This Variable indicates the direction in which the robot faces, useful in helping the robot to determine the next move
# facing_direction="up"

for i in maze:
    print(i)    

for i in range(0,len(path)-1):
    mazeMapping_integration.move_integration_robot(path[i],path[i+1])
    wait(2000)

        