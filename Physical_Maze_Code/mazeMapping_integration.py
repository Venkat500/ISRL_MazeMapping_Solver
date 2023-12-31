def within_bounds(x, y, rows, cols):
    return 0 <= x < cols and 0 <= y < rows

#This is using the move_integration.py to call the move_integrationments instead of throing them all into a single program
import move_integration
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from random import choice
import astar
import Check_maze_skip
# value 0 -> Explored
# value -1 -> Unexplored and accessible (not an obstacle -> Opening)
# value -2 -> Obstacle
rows = 2
cols = 2
maze = [[-1 for _ in range(rows)] for _ in range(cols)]
#This Variable indicates the direction in which the robot faces, useful in helping the robot to determine the next move_integration_integration
facing_direction="up"

def move_integration_robot(current_cell, target_cell):
    global facing_direction

    print("==========================================================")
    print("\n\n\n\nCurrent Position: ", current_cell)
    print("\nNext Position: ", target_cell)
    wait(2000)
    print("facing direction : ",facing_direction)
    if current_cell[0] > target_cell[0]:
        print("Moving UP")
        facing_direction = move_integration.up(facing_direction)
        wait(2000)
    elif current_cell[0] < target_cell[0]:
        print("Moving DOWN")
        facing_direction = move_integration.down(facing_direction)
        wait(2000)
    else:
        print("X axis on same level")
    # wait(4000)

    if current_cell[1] > target_cell[1]:
        print(" Moving LEFT")
        facing_direction = move_integration.left(facing_direction)
        wait(2000)
    elif current_cell[1] < target_cell[1]:
        print("Moving RIGHT")
        facing_direction = move_integration.right(facing_direction)
        wait(2000)
    else:
        print("Y axis on same level")
    

# This function will check whether there are obstacles along the way
# Front - Right - Left
# They'll change according to the robot's facing position.
def check_sensors(x, y):
    print('Front Sensor Reading:', move_integration.check_front_sensor())
    print('Right Sensor Reading:', move_integration.check_right_sensor())
    print('Left Sensor Reading:', move_integration.check_left_sensor())
    print('=================================================================')
    print('\n\n Maze before sensor check:')
    for row in maze:
        print(row)

    if (facing_direction == "up"):
        if move_integration.check_front_sensor() and within_bounds(x - 1, y, rows, cols): 
            print("x : " + str(x-1) + ", y : " + str(y) + " = -2")
            maze[x - 1][y] = -2
        if move_integration.check_right_sensor() and within_bounds(x, y + 1, rows, cols): 
            print("x : " + str(x) + ", y : " + str(y+1) + " = -2")
            maze[x][y + 1] = -2
        if move_integration.check_left_sensor() and within_bounds(x, y - 1, rows, cols): 
            print("x : " + str(x) + ", y : " + str(y-1) + " = -2")
            maze[x][y - 1] = -2
        # rotate to find the missing bottom perspective
        move_integration.turn_right()
        if move_integration.check_right_sensor() and within_bounds(x + 1, y, rows, cols): 
            print("x : " + str(x+1) + ", y : " + str(y) + " = -2")
            maze[x + 1][y] = -2
        wait(2000)
        move_integration.turn_left()
        wait(2000)

    elif (facing_direction == "down"):
        if move_integration.check_front_sensor() and within_bounds(x + 1, y, rows, cols):
            print("x : " + str(x+1) + ", y : " + str(y) + " = -2")
            maze[x + 1][y] = -2
        if move_integration.check_right_sensor() and within_bounds(x, y - 1, rows, cols): 
            print("x : " + str(x) + ", y : " + str(y-1) + " = -2")
            maze[x][y - 1] = -2
        if move_integration.check_left_sensor() and within_bounds(x, y + 1, rows, cols): 
            print("x : " + str(x) + ", y : " + str(y+1) + " = -2")
            maze[x][y + 1] = -2
        # rotate to find the missing bottom perspective
        move_integration.turn_right()
        if move_integration.check_right_sensor() and within_bounds(x - 1, y, rows, cols): 
           print("x : " + str(x-1) + ", y : " + str(y) + " = -2")
           maze[x - 1][y] = -2
        wait(2000)
        move_integration.turn_left()
        wait(2000)

    elif (facing_direction == "left"):
        if move_integration.check_front_sensor() and within_bounds(x, y - 1, rows, cols):
            print("x : " + str(x) + ", y : " + str(y-1) + " = -2")
            maze[x][y - 1] = -2
        if move_integration.check_right_sensor() and within_bounds(x - 1, y, rows, cols): 
            print("x : " + str(x-1) + ", y : " + str(y) + " = -2")
            maze[x - 1][y] = -2
        if move_integration.check_left_sensor() and within_bounds(x + 1, y, rows, cols): 
            print("x : " + str(x+1) + ", y : " + str(y) + " = -2")
            maze[x + 1][y] = -2
        # rotate to find the missing bottom perspective
        move_integration.turn_right()
        if move_integration.check_right_sensor() and within_bounds(x, y + 1 , rows, cols): 
            print("x : " + str(x) + ", y : " + str(y+1) + " = -2")
            maze[x][y + 1] = -2
        wait(2000)
        move_integration.turn_left()
        wait(2000)

    elif (facing_direction == "right"):
        if move_integration.check_front_sensor() and within_bounds(x, y + 1, rows, cols):
            print("x : " + str(x) + ", y : " + str(y+1) + " = -2")
            maze[x][y + 1] = -2
        if move_integration.check_right_sensor() and within_bounds(x + 1, y, rows, cols): 
            print("x : " + str(x+1) + ", y : " + str(y) + " = -2")
            maze[x + 1][y] = -2
        if move_integration.check_left_sensor() and within_bounds(x - 1, y, rows, cols): 
            print("x : " + str(x-1) + ", y : " + str(y) + " = -2")
            maze[x - 1][y] = -2
        # rotate to find the missing bottom perspective
        move_integration.turn_right()
        if move_integration.check_right_sensor() and within_bounds(x, y - 1, rows, cols): 
            print("x : " + str(x) + ", y : " + str(y-1) + " = -2")
            maze[x][y - 1] = -2
        wait(2000)
        move_integration.turn_left()
        wait(2000)
    print('=================================================================')

    print("hi")
    print("Maze after sensor check:")
    for row in maze:
        print(row)
    print('=================================================================')


def checkCell(x, y):
    if x < 0 or x > rows - 1 or y < 0 or y > cols - 1:
        return False
    return True

def visited(x, y):
    if maze[x][y] == -1:
        return False
    return True

# Checks whether there are obstacles - Out of bondary cells - Already explored cells
# The final list will contain unexplored available cells.
# The final choice will be random between those available cells.
def check_neighbors(x, y):
    neighbors = []
    print('=================================================================')
    print('\n\n Maze before sensor check:')
    for row in maze:
        print(row)
    
    check_sensors(x, y)

    if (facing_direction == "up"):
        if checkCell(x, y - 1) and not visited(x, y - 1) and within_bounds(x, y - 1, rows, cols): 
            neighbors.append((x, y - 1)) # Right
        if checkCell(x + 1, y) and not visited(x + 1, y) and within_bounds(x + 1, y, rows, cols): 
            neighbors.append((x + 1, y)) # Back
        if checkCell(x - 1, y) and not visited(x - 1, y) and within_bounds(x - 1, y, rows, cols): 
            neighbors.append((x - 1, y)) # Front
        if checkCell(x, y + 1) and not visited(x, y + 1) and within_bounds(x, y + 1, rows, cols): 
            neighbors.append((x, y + 1)) # Left

    elif (facing_direction == "down"):
        if checkCell(x, y + 1) and not visited(x, y + 1) and within_bounds(x, y + 1, rows, cols): 
            neighbors.append((x, y + 1)) # Left
        if checkCell(x - 1, y) and not visited(x - 1, y) and within_bounds(x - 1, y, rows, cols): 
            neighbors.append((x - 1, y)) # Back
        if checkCell(x + 1, y) and not visited(x + 1, y) and within_bounds(x + 1 , y, rows, cols): 
            neighbors.append((x + 1, y)) # Front
        if checkCell(x, y - 1) and not visited(x, y - 1) and within_bounds(x, y - 1, rows, cols): 
            neighbors.append((x, y - 1)) # Right

    elif (facing_direction == "left"):
        if checkCell(x - 1, y) and not visited(x - 1, y) and within_bounds(x - 1, y, rows, cols): 
            neighbors.append((x - 1, y)) # Right
        if checkCell(x, y - 1) and not visited(x, y - 1) and within_bounds(x, y - 1, rows, cols): 
            neighbors.append((x, y - 1)) # Front
        if checkCell(x, y + 1) and not visited(x, y + 1) and within_bounds(x, y + 1, rows, cols): 
            neighbors.append((x, y + 1)) # Back
        if checkCell(x + 1, y) and not visited(x + 1, y) and within_bounds(x + 1, y, rows, cols): 
            neighbors.append((x + 1, y)) # Left

    elif (facing_direction == "right"):
        if checkCell(x + 1, y) and not visited(x + 1, y) and within_bounds(x + 1, y, rows, cols): 
            neighbors.append((x + 1, y)) # Right
        if checkCell(x, y + 1) and not visited(x, y + 1) and within_bounds(x, y + 1, rows, cols): 
            neighbors.append((x, y + 1)) # Front
        if checkCell(x, y - 1) and not visited(x, y - 1) and within_bounds(x, y - 1, rows, cols): 
            neighbors.append((x, y - 1)) # Back
        if checkCell(x - 1, y) and not visited(x - 1, y) and within_bounds(x - 1, y, rows, cols): 
            neighbors.append((x - 1, y)) # Left
    
    
    print('Maze after neighbour check:')
    for row in maze:
        print(row)
    
    print('\n===================================================================')
    print('\nFound Neighbors:', neighbors)
    if not neighbors:
        print('No neighbors found!')
    print('\n===================================================================')
    return neighbors


def mazeMap():
    current_cell = (0, 0)  # Starting point, inside stack and visited.
    stack = [current_cell]
    maze[current_cell[0]][current_cell[1]] = 0
    
    while len(stack) > 0:
        print('Stack:', stack)
        print('Current Cell:', current_cell)
        
        neighbors = check_neighbors(current_cell[0], current_cell[1])
        
        if not neighbors:  # No unexplored neighbors
            print("No available neighbours so popping")
            print("Stack : ",stack)
            print("Current Cell : ", current_cell)
            start = current_cell
            current_cell = stack.pop()
            print("New Current Cell : ", current_cell)
            move_integration_robot(start, current_cell)
            continue
    
        for next_cell in neighbors:
            if maze[next_cell[0]][next_cell[1]] == -1:  # Only process if it's an unexplored cell
                print("Marking X: ",next_cell[0])
                print("Marking Y: ",next_cell[1])
                print("\n\n Marking them as zero\n")
                maze[next_cell[0]][next_cell[1]] = 0 
                move_integration_robot(current_cell, next_cell)
                current_cell = next_cell
                stack.append(next_cell)

        # to check if the maze is expored completely irrespective of stack 
        if not Check_maze_skip.check_maze(maze):  
            print("Maze fully explored!")

            # Use the A* algorithm to find the path back to the start
            start_cell = current_cell  # Current position of the robot
            goal_cell = (0, 0)  # Assuming the start of the maze is at (0, 0)

            path_to_start = astar.a_star_search(maze, start_cell, goal_cell)

            # Now you have the path, navigate the robot along this path
            for i in maze:
                print(i)    

            for i in range(0,len(path_to_start)-1):
                move_integration_robot(path_to_start[i],path_to_start[i+1])
                # wait(2000)

            break

    if stack:
        current_cell = stack[-1]
 

    wait(2000)

    print("==============================================================")
    print("Going to goal\n------------------------------------------------")
    print("Full maze generated!")
    return maze, facing_direction

