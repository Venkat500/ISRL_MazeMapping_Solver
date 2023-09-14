#!/usr/bin/env pybricks-micropython
import astar

#This is using the move.py to call the movements instead of throing them all into a single program
import move

#This is used to mape the maze 
import mazeMapping


# This is Part 1 of Maze Mapping: the Maze Generator algorithm
# through a DSF pre-order approach.
# It will return a fully explored maze.
# Value 0  -> Approachable cells
# Value -2 -> Obstacles
maze, facing_direction = mazeMapping.mazeMap()

#This is Part 2 of Finding the Shortest Path using A*(A star) algorithm
start = (0, 0)
goal = (2, 2)  # This now directly represents the goal location in the maze

#This is a maze solving Algorithm Called A*star, uses the functions defined in the astar.py
path = astar.a_star_search(maze, start, goal)
print("Answer :", path)

#This Variable indicates the direction in which the robot faces, useful in helping the robot to determine the next move
# facing_direction="up"

for i in maze:
    print(i)    

for i in range(0,len(path)-1):
    mazeMapping.move_robot(path[i],path[i+1])
    # if( path[i][1] > path[i+1][1] ):
    #     print("left")
    #     facing_direction = move.left(facing_direction)
        
    # elif( path[i][1] < path[i+1][1] ):
    #     print("right")
    #     facing_direction = move.right(facing_direction)
        
    # elif( path[i][0] > path[i+1][0] ):
    #     print("up")
    #     facing_direction = move.up(facing_direction)
        
    # else:
    #     print("down")
    #     facing_direction = move.down(facing_direction)
        