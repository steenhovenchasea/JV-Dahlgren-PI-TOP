#import Pi-Top Hardware+controllers
from pitop import Pitop            
from pitop import Display         #tiny screen on pitop
    

from pitop import DriveController    #used to allow pre-calculated drive controls
from pitop import BrakingType        #allows motors to keep position still
from pitop import Direction          #used to differentiate between forwards and backwards driving directions

from pitop import EncoderMotor        #drive motors
from pitop import ServoMotor          #camera Servos
from pitop import UltrasonicSensor    #distance sensor
from pitop import Camera              #camera

#import other libraries        
import time                            #used to wait, and run things for specific times

#current route:
route = "CWsquare"




#constant instantiations
SpinMult=-1/36                         
driveOverShoot=.25
distSensorFootMult=10*3.208            #converts sonar from "meters" to feet
distSensoroffset = 4                    #constant distance that sonar is from wheel front

#variable instantiations
distSensorFeet=0                        #variable tracking sonar difference in feet
absAngle=0                              #total amount we've tried to rotate since the robot turned on

#instantiations
robot = Pitop()                            #allows robot ports to be used
PitchServo = ServoMotor("S0",0,"pitch")    #define pitchservo, port S0, 0 is in the center
YawServo = ServoMotor("S3",0,"yaw")        #define yawservo, port S3, 0 is in the center
distSensor = UltrasonicSensor("D3",5,1)    #define distsensor, Port D3, queue of 5?, 1 meter max
cam = Camera()                             #define cameera
leftMotor= EncoderMotor("M0",Direction.FORWARD,BrakingType.BRAKE,2.75,"LEFTY")#left drive motor, 2.75inchwheel
rightMotor= EncoderMotor("M3",Direction.BACK,BrakingType.BRAKE,2.75,"RIGHT")#right drive motor, 2.75inchwheel
drivebase = DriveController("M0","M3")     #define drivebase, leftmotor M0, right motor M3

drivebase.wheel_separation=6               #wheels are 6 inches apart(used for rotation calcluations)
drivebase.max_robot_angular_speed=.5       #limit max robot rotation speed 
leftMotor.braking_type=BrakingType.BRAKE   #left motor holds position when idle
rightMotor.braking_type=BrakingType.BRAKE  #right motor holds position when idle


cam.capture_image()                        #take a starting postion image


#function to drive exact distance in feet, reccomneded speed <=.5
def driveDist(Dist,speed=.5):
    
    drivebase.forward(speed,False,(abs(Dist)-driveOverShoot)*.3048) 

#previous formula, as fallback
#    if(Dist>0):                                                    #if robot wants to go forwards
#       drivebase.forward(speed,False,(Dist-driveOverShoot)*.3048)  #go forwards, and counteract overshoot
#  if(Dist<0):                                                      #if robot wants ot go backwards
#     drivebase.backward(speed,False,(abs(Dist+driveOverShoot))*.3048)#go forwards, and counteract overshoot

#function to spin, relative to current position
def RelSpin(angle):
    global absAngle                                    #allow absolute angle to update
    drivebase.rotate(angle*SpinMult,abs(angle/90))     #rotate the desired angle, at the same speed every time
    absAngle+=angle                                    #increment the field-centric- absolute angle

#function to spin to an absolute- field centric angle
def absSpin(goalangle):                                
    global absAngle       #allow updates and use of absolute angle
    RelSpin(goalangle-absAngle)     #spin the for delta-theta degrees
    absAngle=goalangle              #reset absolute angle to current angle

#function to easily wait desired seconds
def waitSecs(seconds):             
    time.sleep(seconds)    #litteraly just wait
    
#function to intuitively set the servo positions
def setServos(yaw,pitch):   
    PitchServo.target_angle = -pitch #set pitch to negative parameter(positive is up)
    YawServo.target_angle = -yaw    #set yaw to negative parameter(right)

#function to drive to e specified distance from a wall only accurate 
def driveToDist(distance,margin=.2):
    
    if(abs(distSensorFeet/10-distance)>margin): #if robot is not within an acceptabel range from target wall
        driveDist((distSensorFeet/10)-distance,abs((distSensorFeet/10)-distance)/2)#drive in the correct direction, at a proportional speed
    else:        #if we are within an acceptable threshold
        return True    #return true, can be used to end wating conditions





def auto(autoRoute):
    #auto route drives in a 2 foot square, clockwise, and sets servos to face original forwards, when possible
    if(autoRoute=="CWsquare"):
        driveDist(2)
        relSpin(90)
        setServos(-90,0)
        driveDist(2)
        relSpin(90)
        setServos(0,90)
        driveDist(2)
        relSpin(90)
        setServos(90,0)
        driveDist(2)
        absSpin(0)
        setServos(0,0)
    #auto route drives in a 2 foot square, counterClockwise, and sets servos to face original forwards, when possible
    if(autoRoute=="CCWsquare"):
        driveDist(2)
        relSpin(-90)
        setServos(90,0)
        driveDist(2)
        relSpin(-90)
        setServos(0,90)
        driveDist(2)
        relSpin(-90)
        setServos(-90,0)
        driveDist(2)
        absSpin(0)
        setServos(0,0)

    #test spinning to diffent angles, clockwise first
    if(autoRoute=="CwSpinTest"):
        absSpin(90)
        absSpin(0)
        absSpin(180)
        absSpin(0)
        absSpin(270)
        absSpin(0)
        absSpin(360)
        absSpin(0)
        absSpin(360*5)
        absSpin(0)
    
    #test spinning to diffent angles, counterclockwise first
    if(autoRoute=="CCwSpinTest"):
        absSpin(-90)
        absSpin(0)
        absSpin(-180)
        absSpin(0)
        absSpin(-270)
        absSpin(0)
        absSpin(-360)
        absSpin(0)
        absSpin(-360*5)
        absSpin(0)
        

