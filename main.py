#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from threading import Thread

DRIVE_MODE_FORWARD = 1
DRIVE_MODE_BACKWARD = 2
DRIVE_MODE_LEFT = 3
DRIVE_MODE_RIGHT = 4
DRIVE_MODE_STOP = 5
DRIVE_MODE_UPRIGHT = 6
DRIVE_MODE_UPSIDEDOWN = 7

FORWARD_SPEED = 200
TURN_SPEED = 100
FLIP_SPEED = 200
MOVEMENT_DURATION = 100

class RobotData:
    currentDriveMode = DRIVE_MODE_STOP
    running = False
    startingRot = 0.0
    rotation = 0.0
    xVal = 0
    totalTurnValues = 0
    seesAWall = False
    seesACliff = False
    wantedRotation = 0.0
    hasWantedRotation = False
    isAvoidingObstacles = False

    def __init__(self, _mode, _running, _rotation):
        self.currentDriveMode = _mode
        self.running = _running
        self.startingRot = _rotation

    def SetMode(self, _modeNum):
        self.currentDriveMode = _modeNum
    def SeesObstacles(self):
        return self.seesACliff or self.seesAWall

#Init motors
motorA = Motor(Port.A)
motorB = Motor(Port.B)
motorD = Motor(Port.D)

#Init sensors
ultrasonicDown = UltrasonicSensor(Port.S1)
ultrasonicFront = UltrasonicSensor(Port.S2)
gyrosensor = GyroSensor(Port.S4)
gyrosensor.reset_angle(0)

#Init data container
robotData = RobotData(DRIVE_MODE_STOP, True, motorD.angle())

#Drives the vehicle at a set speed
def Drive(speed):
    motorA.run(speed)
    motorB.run(speed)

#Drives the vehicle at a speed for a time
def DriveTimed(speed, time):
    motorA.run_time(speed, MOVEMENT_DURATION)
    motorB.run_time(speed, MOVEMENT_DURATION)

#Turns the vehicle at a speed
def Turn(turnSpeed):
    motorA.run(-turnSpeed)
    motorB.run(turnSpeed)
    
#Stops the driveing motors    
def Stop():
    motorA.run(0)
    motorB.run(0)

#Flips the brick to the upright state
def FlipUpright(waitValue):
    motorD.run_target(FLIP_SPEED, robotData.startingRot, waitValue)

#Flips brick to upside down state
def FlipUpsideDown(waitValue):
    motorD.run_target(FLIP_SPEED, robotData.startingRot + 190, waitValue)

#Old# Does basic drive logic based on the stored mode
def DriveLogic():
    if (robotData.running):
        if (robotData.currentDriveMode == DRIVE_MODE_FORWARD):
            Drive(FORWARD_SPEED)
        elif (robotData.currentDriveMode == DRIVE_MODE_BACKWARD):
            Drive(-FORWARD_SPEED)
        elif (robotData.currentDriveMode == DRIVE_MODE_LEFT):
            Turn(TURN_SPEED)
        elif (robotData.currentDriveMode == DRIVE_MODE_RIGHT):
            Turn(-TURN_SPEED)
        elif (robotData.currentDriveMode == DRIVE_MODE_UPRIGHT):
            FlipUpright()
        elif (robotData.currentDriveMode == DRIVE_MODE_UPSIDEDOWN):
            FlipUpsideDown()
    else:
        Stop()

#Flips the brick forever
def FlippingLoop():
    while True:
        FlipUpsideDown(True)
        FlipUpright(True)
        wait(1000)

#Rotates the vehicle to the wanted angle
def RotateToStoredRotation():
    currentRot = gyrosensor.angle()
    differenceRot = currentRot - robotData.hasWantedRotation

    print(differenceRot)

    if (differenceRot < 0):
        print("not TURN_SPEED")
        Turn(not TURN_SPEED)
    elif (differenceRot > 0):
        print("TURN_SPEED")
        Turn(TURN_SPEED)
    else:
        print("Stop Turn")
        robotData.hasWantedRotation = False
        robotData.rotation = gyrosensor.angle()

#Checks the sensors and stores the resualts in the relivent variables
def SensorChecks():
    robotData.seesAWall = not (ultrasonicFront.distance() > 100)
    robotData.seesACliff = not (ultrasonicDown.distance() < 70)

#Sets the wanted rotation variables to current rotation plus the amount
def SetWantedRotation(amount):
    robotData.wantedRotation = amount + robotData.rotation
    robotData.hasWantedRotation = True

    if (amount > 0):
        robotData.totalTurnValues -= 1
    elif (amount < 0):
        robotData.totalTurnValues += 1


def main():
    while True:
        SensorChecks()
        if (robotData.hasWantedRotation):
            print("hasWantedRotation")
            RotateToStoredRotation()
        else:
            seesObstacles = robotData.SeesObstacles()
            if (seesObstacles) and (not robotData.isAvoidingObstacles):
                robotData.isAvoidingObstacles = True            
                SetWantedRotation(90)

            if (not seesObstacles):
                Drive(FORWARD_SPEED)
                robotData.isAvoidingObstacles = False
            else:
                Stop()
            DriveLogic()



t = Thread(target = FlippingLoop)
#t.start()

main()