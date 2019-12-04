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
    mode = DRIVE_MODE_STOP
    running = False
    startingRot = 0
    rotation = 0
    xVal = 0
    def __init__(self, _mode, _running, _rotation):
        self.mode = _mode
        self.running = _running
        self.startingRot = _rotation
    def SetMode(self, _modeNum):
        self.mode = _modeNum


mA = Motor(Port.A)
mB = Motor(Port.B)
mD = Motor(Port.D)

sD = UltrasonicSensor(Port.S1)
sF = UltrasonicSensor(Port.S2)
sR = GyroSensor(Port.S4)
sR.reset_angle()

rData = RobotData(DRIVE_MODE_STOP, False, mD.angle())

def Drive(speed):
    mA.run(speed)
    mB.run(speed)

def Drive(speed, time):
    mA.run_time(speed, MOVEMENT_DURATION)
    mB.run_time(speed, MOVEMENT_DURATION)

def Turn(turnSpeed):
    mA.run(-turnSpeed)
    mB.run(turnSpeed)

def Turn(turnSpeed):
    mA.run_time(-turnSpeed, MOVEMENT_DURATION)
    mB.run_time(turnSpeed, MOVEMENT_DURATION)
    
def Stop():
    mA.run(0)
    mB.run(0)

def FlipUpright(waitValue):
    mD.run_target(FLIP_SPEED, rData.startingRot, waitValue)

def FlipUpsideDown(waitValue):
    mD.run_target(FLIP_SPEED, rData.startingRot + 190, waitValue)

def DriveLogic():
    global rData
    if rData.running:
        if rData.mode == DRIVE_MODE_FORWARD:
            Drive(FORWARD_SPEED)
        elif rData.mode == DRIVE_MODE_BACKWARD:
            Drive(-FORWARD_SPEED)
        elif rData.mode == DRIVE_MODE_LEFT:
            Turn(TURN_SPEED)
        elif rData.mode == DRIVE_MODE_RIGHT:
            Turn(-TURN_SPEED)
        elif rData.mode == DRIVE_MODE_UPRIGHT:
            FlipUpright()
        elif rData.mode == DRIVE_MODE_UPSIDEDOWN:
            FlipUpsideDown()
    else:
        Stop()

def DrivingLoop():
    while True:
        FlipUpsideDown(True)
        FlipUpright(True)
        wait(2000)

t = Thread(target = DrivingLoop)
t.start()
isAWall = False
isACliff = False
wantedRotation = 0
hasWantedRotation = False

def RotateTo()
    global hasWantedRotation
    currentRot = sR.angle()
    differenceRot = currentRot - hasWantedRotation

    if differenceRot < 0 or currentRot < -90:
        TurnRight()
    elif differenceRot > 0 or currentRot < 90:
        TurnLeft()
    else
        hasRotation = False
        rData.rotation = sR.angle()
        hasWantedRotation = 0

while True:
    if hasRotation:
        RotateTo()

    canDriveFwd = not (sF.distance() > 100)
    isACliff = not (sD.distance() < 70)

    if isACliff:
        rData.SetMode(DRIVE_MODE_LEFT)
        rData.running = True

        if not hasRotation:
            wantedRotation = 90 + rData.rotation
            hasRotation = True
    elif canDriveFwd:
        rData.SetMode(DRIVE_MODE_FORWARD)
    else:
        rData.SetMode(DRIVE_MODE_STOP)
        rData.running = False
    DriveLogic()