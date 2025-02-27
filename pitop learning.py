from pitop import Pitop
from pitop import EncoderMotor
from pitop import BrakingType
from pitop import Direction
from pitop import ServoMotor
from pitop import DriveController
from pitop import LightSensor
from pitop import Display
from pitop import UltrasonicSensor
from pitop import Camera
import time

SpinMult=-1/36
absAngle=0
driveFooDist=.25
distSensorFootMult=10*3.208
distSensoroffset = 4
distSensorFeet=0
robot = Pitop()
PitchServo = ServoMotor("S0",0,"pitch")
YawServo = ServoMotor("S3",0,"yaw")

distSensor = UltrasonicSensor("D3",5,1)

lightdetector=LightSensor("D7",)
lightdetector.read()
cam = Camera()
cam.capture_image()


leftMotor= EncoderMotor("M0",Direction.FORWARD,BrakingType.BRAKE,2.75,"LEFTY")
rightMotor= EncoderMotor("M3",Direction.BACK,BrakingType.BRAKE,2.75,"RIGHT")
drivebase = DriveController("M0","M3")
drivebase.wheel_separation=6
drivebase.max_robot_angular_speed=.5
leftMotor.braking_type=BrakingType.BRAKE
rightMotor.braking_type=BrakingType.BRAKE
#drivebase.robot_move(1,10,3,10)


def driveDist(Dist,speed):
    if(Dist>0):
        drivebase.forward(speed,False,(Dist-driveFooDist)*.3048)
    if(Dist<0):
        drivebase.backward(speed,False,(abs(Dist+driveFooDist))*.3048)

def RelSpin(angle):
    global absAngle
    #drivebase.rotate((angle+SpinFoo)*SpinMult)
    drivebase.rotate(angle*SpinMult,abs(angle/90))
    absAngle+=angle

def absSpin(goalangle):
    global absAngle
    RelSpin(goalangle-absAngle)
    absAngle=goalangle

def waitSecs(seconds):
    time.sleep(seconds)

def setServos(yaw,pitch):
    PitchServo.target_angle = -pitch
    YawServo.target_angle = -yaw

def driveToDist(distance,margin=.2):
    distance = distance
    if(abs(distSensorFeet/10-distance)>margin):
        driveDist((distSensorFeet/10)-distance,abs((distSensorFeet/10)-distance)/2)
    else:
        return True

absSpin(90)
absSpin(0)
absSpin(180)
absSpin(0)
absSpin(270)
absSpin(0)
absSpin(360)
absSpin(0)
