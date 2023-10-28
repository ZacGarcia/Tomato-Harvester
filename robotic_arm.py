from nanpy import (ArduinoApi, SerialManager, Stepper)
from time import sleep
import gripper

xstepPin = 2 
xdirPin = 5

ystepPin = 3
ydirPin = 6

zstepPin = 4 
zdirPin = 7

step_count = 2048
delay = .01

try:
    connection = SerialManager()
    a = ArduinoApi(connection = connection)
except:
    print("Error connect!")

a.pinMode(xstepPin, a.OUTPUT)
a.pinMode(xdirPin, a.OUTPUT)

a.pinMode(ystepPin, a.OUTPUT)
a.pinMode(ydirPin, a.OUTPUT)

a.pinMode(zstepPin, a.OUTPUT)
a.pinMode(zdirPin, a.OUTPUT)

def calibrate_joints():
    try:
        a.digitalWrite(xdirPin, a.LOW)
        sleep(1)
        x = 0
        while (x < 120):
            a.digitalWrite(xstepPin,a.HIGH)
            sleep(delay)
            a.digitalWrite(xstepPin, a.LOW)
            sleep(delay)
            x = x + 1
        a.digitalWrite(ydirPin, a.HIGH)
        sleep(1)
        y = 0
        while (y < 50):
            a.digitalWrite(ystepPin,a.HIGH)
            sleep(delay)
            a.digitalWrite(ystepPin, a.LOW)
            sleep(delay)
            y = y + 1
        a.digitalWrite(zdirPin, a.HIGH)
        sleep(1)
        z = 0
        while (z < 100):
            a.digitalWrite(zstepPin,a.HIGH)
            sleep(delay)
            a.digitalWrite(zstepPin, a.LOW)
            sleep(delay)
            z = z + 1        
        
    except:
        print('unable to run motor..')


    
    return    

def left_xaxis():
    try:
        a.digitalWrite(xdirPin, a.HIGH)
        sleep(1)
        x = 0
        while (x < 300):
            a.digitalWrite(xstepPin,a.HIGH)
            sleep(delay)
            a.digitalWrite(xstepPin, a.LOW)
            sleep(delay)
            x = x + 1
    except:
        print('unbale to run x axis')
    return

def right_xaxis():
    try:
        a.digitalWrite(xdirPin, a.LOW)
        sleep(1)
        x = 0
        while (x < 300):
            a.digitalWrite(xstepPin,a.HIGH)
            sleep(delay)
            a.digitalWrite(xstepPin, a.LOW)
            sleep(delay)
            x = x + 1
    except:
        print('unbale to run x axis')
    return

def forward_yaxis():
    try:
        a.digitalWrite(ydirPin, a.HIGH)
        sleep(1)
        y = 0
        while (y < 150):
            a.digitalWrite(ystepPin,a.HIGH)
            sleep(delay)
            a.digitalWrite(ystepPin, a.LOW)
            sleep(delay)
            y = y + 1
        
    except:
        print('unbale to run y axis')
      
    return
def backward_yaxis():
    try:
        a.digitalWrite(ydirPin, a.LOW)
        sleep(1)
        y = 0
        while (y < 150):
            a.digitalWrite(ystepPin,a.HIGH)
            sleep(delay)
            a.digitalWrite(ystepPin, a.LOW)
            sleep(delay)
            y = y + 1
    except:
        print('unbale to run y axis')
    return 

def up_zaxis():
    try:
        a.digitalWrite(zdirPin, a.HIGH)
        sleep(1)
        z = 0
        while (z < 80):
            a.digitalWrite(zstepPin,a.HIGH)
            sleep(delay)
            a.digitalWrite(zstepPin, a.LOW)
            sleep(delay)
            z = z + 1     
    except:
        print('unable to run z axis')
    return

def down_zaxis():
    try:
        a.digitalWrite(zdirPin, a.LOW)
        sleep(1)
        z = 0
        while (z < 80):
            a.digitalWrite(zstepPin,a.HIGH)
            sleep(delay)
            a.digitalWrite(zstepPin, a.LOW)
            sleep(delay)
            z = z + 1     
    except:
        print('unable to run z axis')
    return

def harvest():
    gripper.open_gripper()
    sleep(2)
    forward_yaxis()
    sleep(2)
    gripper.close_gripper()
    sleep(2)
    backward_yaxis()
    sleep(2)
    left_xaxis()
    sleep(2)
    gripper.open_gripper()
    sleep(2)
    gripper.close_gripper()
    sleep(2)
    right_xaxis()
    return

#GPIO.clean()