from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# from queue import PriorityQueue


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the Ultrasonic Sensors. They're used to detect
# obstacles as the robot drives around.
obstacle_sensor_right = UltrasonicSensor(Port.S3)
obstacle_sensor_left = UltrasonicSensor(Port.S1)
obstacle_sensor_front = UltrasonicSensor(Port.S2)
gyro = GyroSensor(Port.S4)
# Initialize two motors with default settings on Port B and Port C.
# These will be the left and right motors of the drive base.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors      
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

#Check Front Sensor:
def check_front_sensor():
    return True if obstacle_sensor_front.distance() < 150 else False

#Check Right Sensor:
def check_right_sensor():
    return True if obstacle_sensor_right.distance() < 150 else False

#Check Left Sensor:
def check_left_sensor():
    return True if obstacle_sensor_left.distance() < 150 else False

#This is tuned witht the gyroscope
def turn_right():
    gyro.reset_angle(0)
    robot.turn(90)
    print("Angle Gyro: ", gyro.angle())
    while (gyro.angle()<90):
            # print("Turning right")
            robot.turn(90 - gyro.angle()) 
    while(gyro.angle()>90):
            print(" Before left Angle Gyro: ", gyro.angle())
            # print("turning left")
            robot.turn(90 - gyro.angle())
            
#This is tuned witht the gyroscope    
def turn_left():
    gyro.reset_angle(0)
    robot.turn(-90)
    print("Angle Gyro: ", gyro.angle())
    while (gyro.angle() > -90):
            # print("Turning right")
            robot.turn(-90 - gyro.angle()) 
    while(gyro.angle() < -90):
            print(" Before Left Angle Gyro: ", gyro.angle())
            # print("turning left")
            robot.turn(-90 - gyro.angle())

#Driving Right
def right(facing_direction):
    if(facing_direction=="up"):
        turn_right()
        wait(2000)
     
    if(facing_direction=="down"):
        turn_left()
        wait(2000)

    if(facing_direction=="left"):
        turn_left()
        wait(2000)
        turn_left()
        wait(2000)
    move_forward()
    wait(2000)
    return "right"
    

#Driving Left
def left(facing_direction):
    if(facing_direction=="up"):
        turn_left()
        wait(2000)

    if(facing_direction=="down"):
        turn_right()
        wait(2000)

    if(facing_direction=="right"):
        turn_left()
        wait(2000)
        turn_left()
        wait(2000)
    move_forward()
    wait(2000)
    return "left"

#Driving UP
def up(facing_direction):
    if(facing_direction=="right"):
        turn_left()
        wait(2000)

    if(facing_direction=="left"):
        turn_right()
        wait(2000)
    
    if(facing_direction=="down"):
        turn_left()
        wait(2000)
        turn_left()
        wait(2000)
    move_forward()
    wait(2000)
    return "up"


#Driving Down
def down(facing_direction):
    if(facing_direction=="left"):
        turn_left()
        wait(2000)

    if(facing_direction=="right"):
        turn_right()
        wait(2000)
       
    
    if(facing_direction=="up"):
        turn_left()
        wait(2000)
        turn_left()
        wait(2000)

    move_forward()
    wait(2000)
    return "down"

#Moves in a straight line with Gyro Correction
def move_forward():
    robot.reset()
    gyro.reset_angle(0)
    while robot.distance() <= 90:
        # Here the Correction Formula = Error * Kp 
        # Kp means "Gyro Straight Proportional Constant" indicated how aggresively do you want the robot to go back to the straight line
        # 1 means it will be a smootht correction
        # 3 means it will correct and turn abruptly
        correction = (0 - gyro.angle()) * 1
        robot.drive(90, correction)
    # robot.stop()