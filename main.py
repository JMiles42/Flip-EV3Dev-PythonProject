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
FORWARD_SPEED = 400
TURN_SPEED = 100

class RobotData:
    mode = DRIVE_MODE_STOP
    running = False
    startingRot = 0
    def __init__(self, _mode, _running, _rotation):
        self.mode = _mode
        self.running = _running
        self.startingRot = _rotation
    def SetMode(self, _modeNum):
        self.mode = _modeNum


mA = Motor(Port.A)
mB = Motor(Port.B)
mD = Motor(Port.D)

rData = RobotData(DRIVE_MODE_STOP, False, mD.angle())

def Drive(speed):
    mA.run(speed)
    mB.run(speed)

def Turn(turnSpeed):
    mA.run(-turnSpeed)
    mB.run(turnSpeed)
    
def Stop():
    mA.run(0)
    mB.run(0)

def DriveButtons():
    global rData
    buttons = brick.buttons()
    if Button.CENTER in buttons:
        rData.running = not rData.running
    if Button.UP in buttons:
        rData.SetMode(DRIVE_MODE_FORWARD)
    if Button.DOWN in buttons:
        rData.SetMode(DRIVE_MODE_BACKWARD)
    if Button.LEFT in buttons:
        rData.SetMode(DRIVE_MODE_UPSIDEDOWN)
    if Button.RIGHT in buttons:
        rData.SetMode(DRIVE_MODE_UPRIGHT)

def FlipUpright(waitValue):
    mD.run_target(400, rData.startingRot, waitValue)
def FlipUpsideDown(waitValue):
    mD.run_target(400, rData.startingRot + 190, waitValue)

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
        #DriveLogic()
        FlipUpsideDown(True)
        FlipUpright(True)

t = Thread(target = DrivingLoop)
t.start()

while True:
    print("RUNNING")
    #Drive(FORWARD_SPEED)
    #DriveButtons()