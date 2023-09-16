"""e-puck_maze-solver controller."""

# Needed classes - Controller module
from controller import Robot, DistanceSensor, Motor
from controller import Supervisor
from controller import Node

# create the Robot instance.
robot = Robot()
supervisor = Supervisor()
robotSuper = supervisor.getFromDef("robot")

obstacle_sensor_right = robot.getDevice("ds_right")
obstacle_sensor_left = robot.getDevice("ds_left")
obstacle_sensor_front = robot.getDevice("ds_front")

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

timestep = int(robot.getBasicTimeStep())

# Needed to provide movements positions:
start_cell_global = (0,0)
target_cell_global = (0,0)
target_position = 0



# ASTAR ALGORITHM

class SimplePriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        self.elements.append((priority, item))
        self.elements.sort(reverse=True)  # Sort by priority

    def get(self):
        return self.elements.pop()[1]  # Return the item with highest priority

def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def a_star_search(grid, start, goal):
    open_list = SimplePriorityQueue()
    open_list.put(start, 0)
    
    came_from = {start: None}
    g_score = {start: 0}

    while not open_list.empty():
        current = open_list.get()

        if current == goal:  # Directly check if the current node is the goal
            return reconstruct_path(came_from, start, goal)

        for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
            x, y = current[0] + dx, current[1] + dy

            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != -2:
                neighbor = (x, y)
                tentative_g_score = g_score.get(neighbor, float('inf'))
                new_g_score = g_score[current] + 1

                if new_g_score < tentative_g_score:
                    came_from[neighbor] = current
                    g_score[neighbor] = new_g_score
                    priority = new_g_score + heuristic(neighbor, goal)
                    open_list.put(neighbor, priority)

    return None

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []

    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path
    




# MOVE

#Check Front Sensor:
def check_front_sensor():
    return True if obstacle_sensor_front.getValue() < 1000 else False

#Check Right Sensor:
def check_right_sensor():
    return True if obstacle_sensor_right.getValue() < 1000 else False

#Check Left Sensor:
def check_left_sensor():
    return True if obstacle_sensor_left.getValue() < 1000 else False

# Support variables for turning around
isTurning = False
target_rotation = 0



def turn_right():
    global target_rotation
    
    if robotSuper.getField('rotation').getSFVec3f()[2] > 0:
        if round(robotSuper.getField('rotation').getSFVec3f()[3], 1) == 0:
            angle = 0
        else:
            angle = round(robotSuper.getField('rotation').getSFVec3f()[3], 2) 
            
        wanted_rotation = angle - 1.57
        target_rotation = robotSuper.getField('rotation').getSFVec3f()[3] - 1.57
    
        if target_rotation < wanted_rotation or target_rotation > wanted_rotation:
            difference = wanted_rotation - target_rotation
            target_rotation += difference
            
        print("W", wanted_rotation, "T", target_rotation)
    else:
        if round(robotSuper.getField('rotation').getSFVec3f()[3], 1) == 0:
            angle = 0
        else:
            angle = round(robotSuper.getField('rotation').getSFVec3f()[3], 2) 
            
        wanted_rotation = angle + 1.57
        target_rotation = robotSuper.getField('rotation').getSFVec3f()[3] + 1.57
    
        if target_rotation < wanted_rotation or target_rotation > wanted_rotation:
            difference = wanted_rotation - target_rotation
            target_rotation += difference
            
        print("W", wanted_rotation, "T", target_rotation)

    
    left_motor.setVelocity(0.25)
    right_motor.setVelocity(-0.25)
    
    if target_rotation < -3.14:
        target_rotation += 6.28
    if target_rotation > 3.14:
        target_rotation -= 6.28
        
    target_rotation = round(target_rotation, 2)

    print("RIGHT ROT TARGET" , target_rotation)
            
            
            
def turn_left():
    global target_rotation
        
    if robotSuper.getField('rotation').getSFVec3f()[2] > 0:        
        if round(robotSuper.getField('rotation').getSFVec3f()[3], 1) == 0:
            angle = 0
        else:
            angle = round(robotSuper.getField('rotation').getSFVec3f()[3], 2) 
            
        wanted_rotation = angle + 1.57
        target_rotation = robotSuper.getField('rotation').getSFVec3f()[3] + 1.57
    
        if target_rotation < wanted_rotation or target_rotation > wanted_rotation:
            difference = wanted_rotation - target_rotation
            target_rotation += difference
            
        print("W", wanted_rotation, "T", target_rotation)
    else:
        if round(robotSuper.getField('rotation').getSFVec3f()[3], 1) == 0:
            angle = 0
        else:
            angle = round(robotSuper.getField('rotation').getSFVec3f()[3], 2) 
            
        wanted_rotation = angle - 1.57
        target_rotation = robotSuper.getField('rotation').getSFVec3f()[3] - 1.57
    
        if target_rotation < wanted_rotation or target_rotation > wanted_rotation:
            difference = wanted_rotation - target_rotation
            target_rotation += difference
            
        print("W", wanted_rotation, "T", target_rotation)
    
    left_motor.setVelocity(-0.25)
    right_motor.setVelocity(0.25)
    
    if target_rotation > 3.14:
        target_rotation -= 6.28
    if target_rotation < -3.14:
        target_rotation += 6.28
        
    target_rotation = round(target_rotation, 2)
        
    print("LEFT ROT TARGET" , target_rotation)



def turn_around():
    global target_rotation
    
    if round(robotSuper.getField('rotation').getSFVec3f()[3], 1) == 0:
        angle = 0
    else:
        angle = round(robotSuper.getField('rotation').getSFVec3f()[3], 2) 
    
    wanted_rotation = angle + 3.14
    target_rotation = robotSuper.getField('rotation').getSFVec3f()[3] + 3.14
    
    if target_rotation < wanted_rotation:
        difference = wanted_rotation - target_rotation
        target_rotation += difference
    print("W", wanted_rotation, "T", target_rotation)
    
    left_motor.setVelocity(-0.25)
    right_motor.setVelocity(0.25)
    
    if target_rotation > 3.14:
        target_rotation -= 6.28

    target_rotation = round(target_rotation, 2)

    print("AROUND ROT TARGET" , target_rotation)


#Driving Right
def right(facing_direction):
    global start_cell_global
    global target_cell_global
    global target_chosen
    global target_rotation
    global isTurning
    global target_position
    
    print(isTurning, " posizione ", robotSuper.getField('rotation').getSFVec3f()[3], " ", target_position)
    
    if isTurning and (round(robotSuper.getField('rotation').getSFVec3f()[3],2) == target_rotation or round(robotSuper.getField('rotation').getSFVec3f()[3],2) == -target_rotation):
        isTurning = False

    if(facing_direction=="up"):
        isTurning = True
        turn_right()
     
    if(facing_direction=="down"):
        isTurning = True
        turn_left()

    if(facing_direction=="left"):
        isTurning = True     
        turn_around()

    if(not isTurning and facing_direction=="right"):
        left_motor.setVelocity(4.0)
        right_motor.setVelocity(4.0)
        if round(robotSuper.getField('translation').getSFVec3f()[0],2) == round(float(target_position), 2):
            print(start_cell_global, target_cell_global)
            start_cell_global = target_cell_global
            target_chosen = False

    return "right"
    


#Driving Left
def left(facing_direction):
    global start_cell_global
    global target_cell_global
    global target_chosen
    global target_rotation
    global isTurning
    global target_position
    
    print(isTurning, " posizione ", robotSuper.getField('rotation').getSFVec3f()[3], " ", target_position)
        
    if isTurning and (round(robotSuper.getField('rotation').getSFVec3f()[3],2) == target_rotation or round(robotSuper.getField('rotation').getSFVec3f()[3],2) == -target_rotation):        
        isTurning = False

    if(facing_direction=="up"):
        isTurning = True
        turn_left()
     
    if(facing_direction=="down"):
        isTurning = True
        turn_right()

    if(facing_direction=="right"):
        isTurning = True
        turn_around()
        
    if(not isTurning and facing_direction=="left"):
        left_motor.setVelocity(4.0)
        right_motor.setVelocity(4.0)
        if round(robotSuper.getField('translation').getSFVec3f()[0],2) == round(float(target_position), 2):
            print(start_cell_global, target_cell_global)
            start_cell_global = target_cell_global
            target_chosen = False

    return "left"



#Driving UP
def up(facing_direction):
    global start_cell_global
    global target_cell_global
    global target_chosen
    global target_rotation
    global isTurning
    global target_position
    global hasRotated
    
    print(isTurning, hasRotated, " posizione ", robotSuper.getField('rotation').getSFVec3f()[3], " ", target_position)
    
    if isTurning and not hasRotated and (round(robotSuper.getField('rotation').getSFVec3f()[3], 1) == 0 or round(robotSuper.getField('rotation').getSFVec3f()[3], 2) == 3.14 or round(robotSuper.getField('rotation').getSFVec3f()[3], 2) == -3.14):
        hasRotated = True
    
    if isTurning and hasRotated and (round(robotSuper.getField('rotation').getSFVec3f()[3],2) == target_rotation or round(robotSuper.getField('rotation').getSFVec3f()[3],2) == -target_rotation):        
        isTurning = False
        hasRotated = False

    if(facing_direction=="right"):
        isTurning = True
        hasRotated = True
        turn_left()

    if(facing_direction=="left"):
        isTurning = True
        hasRotated = True
        turn_right()
        
    if(facing_direction=="down"):
        isTurning = True
        hasRotated = False
        turn_around()

    if(not isTurning and facing_direction=="up"):
        left_motor.setVelocity(4.0)
        right_motor.setVelocity(4.0)
        if round(robotSuper.getField('translation').getSFVec3f()[1],2) == round(float(target_position), 2):
            print(start_cell_global, target_cell_global)
            start_cell_global = target_cell_global
            target_chosen = False
    
    return "up"

hasRotated = False

#Driving Down
def down(facing_direction):
    global start_cell_global
    global target_cell_global
    global target_chosen
    global target_rotation
    global isTurning
    global target_position
    global hasRotated
    
    print(isTurning, hasRotated, " posizione ", robotSuper.getField('rotation').getSFVec3f()[3], " ", target_position)
    
    if isTurning and not hasRotated and (round(robotSuper.getField('rotation').getSFVec3f()[3], 1) == 0 or round(robotSuper.getField('rotation').getSFVec3f()[3], 2) == 3.14 or round(robotSuper.getField('rotation').getSFVec3f()[3], 2) == -3.14):
        hasRotated = True
    
    if isTurning and hasRotated and (round(robotSuper.getField('rotation').getSFVec3f()[3],2) == target_rotation or round(robotSuper.getField('rotation').getSFVec3f()[3],2) == -target_rotation):        
        isTurning = False
        hasRotated = False
        
    if(facing_direction=="left"):
        isTurning = True
        hasRotated = True
        turn_left()

    if(facing_direction=="right"):
        isTurning = True
        hasRotated = True
        turn_right()
        
    if(facing_direction=="up"):
        isTurning = True
        hasRotated = False
        turn_around()

    if(not isTurning and facing_direction=="down"):
        left_motor.setVelocity(4.0)
        right_motor.setVelocity(4.0)
        if round(robotSuper.getField('translation').getSFVec3f()[1],2) == round(float(target_position), 2):
            print(start_cell_global, target_cell_global)
            start_cell_global = target_cell_global
            target_chosen = False

    return "down"
    
    
   

# MAZE MAPPING


def within_bounds(x, y, rows, cols):
    return 0 <= x < cols and 0 <= y < rows

# value 0 -> Explored
# value -1 -> Unexplored and accessible (not an obstacle -> Opening)
# value -2 -> Obstacle
rows = 4
cols = 4
maze = [[-1 for _ in range(rows)] for _ in range(cols)]
#This Variable indicates the direction in which the robot faces, useful in helping the robot to determine the next move
facing_direction="up"
target_chosen=False

def move_robot(current_cell, target_cell):
    global facing_direction
    global target_chosen
    global target_position

    print("==========================================================")
    print("\n\n\n\nCurrent Position: ", current_cell)
    print("\nNext Position: ", target_cell)
    
    print("facing direction : ",facing_direction)
    print(target_chosen)
    print(target_position)
    if current_cell[0] > target_cell[0]:
        print("Moving UP")
        if not target_chosen:
            target_position = 0
            target_position = float(robotSuper.getField('translation').getSFVec3f()[1]) + 0.25
            target_chosen = True
        facing_direction = up(facing_direction)
    elif current_cell[0] < target_cell[0]:
        print("Moving DOWN")
        if not target_chosen:
            target_position = 0
            target_position = float(robotSuper.getField('translation').getSFVec3f()[1]) - 0.25
            target_chosen = True
        facing_direction = down(facing_direction)
    else:
        print("X axis on same level")
    # wait(4000)

    if current_cell[1] > target_cell[1]:
        print(" Moving LEFT")
        if not target_chosen:
            target_position = 0
            target_position = float(robotSuper.getField('translation').getSFVec3f()[0]) - 0.25
            target_chosen = True
        facing_direction = left(facing_direction)
    elif current_cell[1] < target_cell[1]:
        print("Moving RIGHT")
        if not target_chosen:
            target_position = 0
            target_position = float(robotSuper.getField('translation').getSFVec3f()[0]) + 0.25
            target_chosen = True 
        facing_direction = right(facing_direction)
    else:
        print("Y axis on same level")
    

# This function will check whether there are obstacles along the way
# Front - Right - Left
# They'll change according to the robot's facing position.
def check_sensors(x, y):
    print('Front Sensor Reading:', check_front_sensor())
    print('Right Sensor Reading:', check_right_sensor())
    print('Left Sensor Reading:', check_left_sensor())
    print('=================================================================')
    print('\n\n Maze before sensor check:')
    for row in maze:
        print(row)

    if (facing_direction == "up"):
        if check_front_sensor() and within_bounds(x-1, y, rows, cols): 
            print("x : " + str(x-1) + ", y : " + str(y) + " = -2")
            maze[x-1][y] = -2
        if check_right_sensor() and within_bounds(x, y+1, rows, cols): 
            print("x : " + str(x) + ", y : " + str(y+1) + " = -2")
            maze[x][y+1] = -2
        if check_left_sensor() and within_bounds(x, y-1, rows, cols): 
            print("x : " + str(x) + ", y : " + str(y-1) + " = -2")
            maze[x][y-1] = -2

    elif (facing_direction == "down"):
        if check_front_sensor() and within_bounds(x+1, y, rows, cols):
            print("x : " + str(x+1) + ", y : " + str(y) + " = -2")
            maze[x+1][y] = -2
        if check_right_sensor() and within_bounds(x, y-1, rows, cols): 
            print("x : " + str(x) + ", y : " + str(y-1) + " = -2")
            maze[x][y-1] = -2
        if check_left_sensor() and within_bounds(x, y+1, rows, cols): 
            print("x : " + str(x) + ", y : " + str(y+1) + " = -2")
            maze[x][y+1] = -2        

    elif (facing_direction == "left"):
        if check_front_sensor() and within_bounds(x, y-1, rows, cols):
            print("x : " + str(x) + ", y : " + str(y-1) + " = -2")
            maze[x][y-1] = -2
        if check_right_sensor() and within_bounds(x-1, y, rows, cols): 
            print("x : " + str(x-1) + ", y : " + str(y-1) + " = -2")
            maze[x-1][y] = -2
        if check_left_sensor() and within_bounds(x+1, y, rows, cols): 
            print("x : " + str(x+1) + ", y : " + str(y) + " = -2")
            maze[x+1][y] = -2

    elif (facing_direction == "right"):
        if check_front_sensor() and within_bounds(x, y+1, rows, cols):
            print("x : " + str(x) + ", y : " + str(y+1) + " = -2")
            maze[x][y+1] = -2
        if check_right_sensor() and within_bounds(x+1, y, rows, cols): 
            print("x : " + str(x+1) + ", y : " + str(y) + " = -2")
            maze[x+1][y] = -2
        if check_left_sensor() and within_bounds(x-1, y, rows, cols): 
            print("x : " + str(x-1) + ", y : " + str(y) + " = -2")
            maze[x-1][y] = -2
        
    print('=================================================================')

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
def check_neighbors(x, y):
    neighbors = []
    print('=================================================================')
    print('\n\n Maze before sensor check:')
    for row in maze:
        print(row)
    
    check_sensors(x, y)

    # For backtracking, it'll happend the cell itself after each neighbor,
    # except the last one which will be explored up first -> pop
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
    
    print("NEIGH", neighbors)
        
    print('Maze after neighbour check:')
    for row in maze:
        print(row)
    
    print('\n===================================================================')
    print('\nFound Neighbors:', neighbors)
    if not neighbors:
        print('No neighbors found!')
    print('\n===================================================================')
    return neighbors

def order_robot(pos_start, pos_goal, maze):
    #This is a maze solving Algorithm Called A*star, uses the functions defined in the astar.py
    path = a_star_search(maze, pos_start, pos_goal)
    print(path)

    #This Variable indicates the direction in which the robot faces, useful in helping the robot to determine the next move
    # facing_direction="up"

    for i in maze:
        print(i)    

    for i in range(0,len(path)-1):

        if( path[i][1] > path[i+1][1] ):
            print("left")
            facing_direction = left(facing_direction)
            
        elif( path[i][1] < path[i+1][1] ):
            print("right")
            facing_direction = right(facing_direction)
            
        elif( path[i][0] > path[i+1][0] ):
            print("up")
            facing_direction = up(facing_direction)
            
        else:
            print("down")
            facing_direction = down(facing_direction)

def adjacencyCheck(start_point, end_point):
    if start_point[0] != end_point[0] and start_point[1] != end_point[1]:
        return False
    else:
        return True


def mazeMap():
    global start_cell_global
    global target_cell_global

    current_cell = (3, 3)  # Starting point, inside stack and visited.
    
    stack = [current_cell]
    path = [current_cell]
    
    maze[current_cell[0]][current_cell[1]] = 0
    
    movimento = False
    
    neighbors = []
    
    while robot.step(timestep) != -1:
        if not movimento and len(stack) == 0:
            break
            
        if not movimento:
            print('Stack:', stack)
            print('Current Cell:', current_cell)
            
            neighbors = check_neighbors(current_cell[0], current_cell[1])
            
            for next_cell in neighbors:            
                if maze[next_cell[0]][next_cell[1]] == -1:  
                # Only process if it's an unexplored cell
                    target_cell_global = next_cell
                    start_cell_global = current_cell
                    #move_robot(current_cell, next_cell)
                    #current_cell = next_cell
                    stack.append(next_cell)
                    stack.append(current_cell)
                    movimento = True
                if maze[next_cell[0]][next_cell[1]] != -1:
                    neighbors.remove(next_cell)
                    
            if len(neighbors) > 0:
                stack.pop()  
        
            if not neighbors:  # No unexplored neighbors
                print("No available neighbours so popping")
                print("Current Cell : ", current_cell)
                print("PATH " , path)
                start = current_cell
                current_cell = stack.pop()
                path_next = path.pop()
                while len(stack) > 0 and not adjacencyCheck(start, current_cell) and not (path_next == current_cell):
                    current_cell = stack.pop()
                    path_next = path.pop()
                print("New Current Cell : ", current_cell)
                start_cell_global = start
                #move_robot(start, current_cell)
                target_cell_global = current_cell
                movimento = True    

        if start_cell_global == target_cell_global:
            if path.count(start_cell_global) == 0:
                path.append(start_cell_global)
            movimento = False
            current_cell = target_cell_global
            maze[current_cell[0]][current_cell[1]] = 0
            continue
        print("PATH " , path)
        print(start_cell_global, target_cell_global)
        move_robot(start_cell_global, target_cell_global)
    

    print("==============================================================")
    print("Going to goal\n------------------------------------------------")
    print("Full maze generated!")
    return maze, facing_direction



def traceFinalPath(path):
    global start_cell_global
    global target_cell_global

    path.reverse()
    stack = path
    path.pop()
    print("Path stack: " , stack)
    
    movimento = False
    
    while robot.step(timestep) != -1:            
        if not movimento and len(stack) == 0:
            break
            
        print("SC" , start_cell_global, "EC", target_cell_global)
        
        if start_cell_global == target_cell_global:
            movimento = False            
            if len(stack) > 0:
                target_cell_global = stack.pop()
                movimento = True
            
        move_robot(start_cell_global, target_cell_global)
    
 
# -- MAIN LOGIC --

if __name__ == "__main__":
    obstacle_sensor_right.enable(timestep)
    obstacle_sensor_left.enable(timestep)
    obstacle_sensor_front.enable(timestep)
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(4.0)
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(4.0)
    
    # This is Part 1 of Maze Mapping: the Maze Generator algorithm
    # through a DSF pre-order approach.
    # It will return a fully explored maze.
    # Value 0  -> Approachable cells
    # Value -2 -> Obstacles
    maze, facing_direction = mazeMap()
    
    #This is Part 2 of Finding the Shortest Path using A*(A star) algorithm
    start = (3, 3)
    goal = (0, 0)  # This now directly represents the goal location in the maze
    
    #This is a maze solving Algorithm Called A*star, uses the functions defined in the astar.py
    path = a_star_search(maze, start, goal)
    print("Answer :", path)
    
    #This Variable indicates the direction in which the robot faces, useful in helping the robot to determine the next move
    # facing_direction="up"
    
    for i in maze:
        print(i)    
    
    if path:
        traceFinalPath(path)
        print("MAZE WAS SOLVED!!!")
        left_motor.setVelocity(0)
        right_motor.setVelocity(0) 
    else:
        print("No path found");
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)