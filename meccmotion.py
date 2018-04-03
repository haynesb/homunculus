#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import serial
from time import sleep

# Motors
# Init:
# DPM Pin Output
# DPM Pin Output

# AW Pin Value Time
# DW Pin 0
# DW Pin 1

# A,D,D
# 3,4,5
# 6,7,8

# rarm_pin 13
# larm_pin 11

class MECCMotion():
    def __init__(self, device='/dev/ttyUSB0', baud=57600,
                 rarm_pin=13, larm_pin=11):
        self.serial = None
        self.device = device
        self.baud = baud
        self.rarm = rarm_pin
        self.larm = larm_pin
        self.lmotor = [3, 4, 5]
        self.rmotor = [6, 7, 8]
        self.led_eyes = 9

    def send(self, command):
        print(command)
        command = command.encode('utf-8')
        self.serial.write(command)
        sleep(0)
        response = self.serial.readline()
        print(response)

    def initialize(self):
        self.serial = serial.Serial(self.device, self.baud)
        sleep(4)
        self.send('V\n')
        self.send('MNI {}\n'.format(self.led_eyes))
        self.send('MNI {}\n'.format(self.rarm))
        self.send('MNI {}\n'.format(self.larm))
        self.send('DPM {} O\n'.format(self.lmotor[1]))
        self.send('DPM {} O\n'.format(self.lmotor[2]))
        self.send('DPM {} O\n'.format(self.rmotor[1]))
        self.send('DPM {} O\n'.format(self.rmotor[2]))
        self.eye_color(0,0,7,4)


    def forward(self, duration=1):
        command = 'DW {} 1\n'.format(self.lmotor[2])
        command += 'DW {} 1\n'.format(self.rmotor[2])
        command += 'AW {} 200 1\n'.format(self.lmotor[0])
        command += 'AW {} 200 1\n'.format(self.rmotor[0])
        self.send(command)
        if duration is None:
            return
        sleep(duration)
        command = 'AW {} 0 1\n'.format(self.lmotor[0])
        command += 'AW {} 0 1\n'.format(self.rmotor[0])
        command += 'DW {} 0\n'.format(self.lmotor[2])
        command += 'DW {} 0\n'.format(self.rmotor[2])
        self.send(command)

    def reverse(self, duration=1):
        command = 'DW {} 1\n'.format(self.lmotor[1])
        command += 'DW {} 1\n'.format(self.rmotor[1])
        command += 'AW {} 200 1\n'.format(self.lmotor[0])
        command += 'AW {} 200 1\n'.format(self.rmotor[0])
        self.send(command)
        if duration is None:
            return
        sleep(duration)
        command = 'AW {} 0 1\n'.format(self.lmotor[0])
        command += 'AW {} 0 1\n'.format(self.rmotor[0])
        command += 'DW {} 0\n'.format(self.lmotor[1])
        command += 'DW {} 0\n'.format(self.rmotor[1])
        self.send(command)

    def turnright(self, angle=90):
        command = 'DW {} 1\n'.format(self.lmotor[2])
        command += 'DW {} 1\n'.format(self.rmotor[1])
        command += 'AW {} 150 1\n'.format(self.lmotor[0])
        command += 'AW {} 150 1\n'.format(self.rmotor[0])
        self.send(command)
        if angle is None:
            return
        sleep(angle/90.0)
        command = 'AW {} 0 1\n'.format(self.lmotor[0])
        command += 'AW {} 0 1\n'.format(self.rmotor[0])
        command += 'DW {} 0\n'.format(self.lmotor[2])
        command += 'DW {} 0\n'.format(self.rmotor[1])
        self.send(command)

    def turnleft(self, angle=90):
        command = 'DW {} 1\n'.format(self.lmotor[1])
        command += 'DW {} 1\n'.format(self.rmotor[2])
        command += 'AW {} 150 1\n'.format(self.lmotor[0])
        command += 'AW {} 150 1\n'.format(self.rmotor[0])
        self.send(command)
        if angle is None:
            return
        sleep(angle/90.0)
        command = 'AW {} 0 1\n'.format(self.lmotor[0])
        command += 'AW {} 0 1\n'.format(self.rmotor[0])
        command += 'DW {} 0\n'.format(self.lmotor[1])
        command += 'DW {} 0\n'.format(self.rmotor[2])
        self.send(command)

    def stopbase(self):
        command = 'AW {} 0 1\n'.format(self.lmotor[0])
        command += 'AW {} 0 1\n'.format(self.rmotor[0])
        command += 'DW {} 0\n'.format(self.lmotor[1])
        command += 'DW {} 0\n'.format(self.lmotor[2])
        command += 'DW {} 0\n'.format(self.rmotor[1])
        command += 'DW {} 0\n'.format(self.rmotor[2])
        self.send(command)

    def rightarm(self, shoulder_pos=90, elbow_pos=90, shoulder_rate=750, elbow_rate=750):
        pin = self.rarm
        self.move_arm(pin, shoulder_pos, elbow_pos, shoulder_rate, elbow_rate)
        
    def leftarm(self, shoulder_pos=90, elbow_pos=90, shoulder_rate=750, elbow_rate=750):
        pin = self.larm
        shoulder_pos = 240 - shoulder_pos
        elbow_pos = 240 - elbow_pos
        self.move_arm(pin, shoulder_pos, elbow_pos, shoulder_rate, elbow_rate)


    def move_arm(self, pin, shoulder_pos, elbow_pos, shoulder_rate, elbow_rate):
        self.send('MNSSMP {0} {1} {2} {3}\nMNSSMP {4} {5} {6} {7}\n'.format(pin, 0,
                                          shoulder_pos, shoulder_rate, pin, 1,
                                          elbow_pos, elbow_rate))

    def eye_color(self, red, green, blue, ramp_time=0):
        """
        red: 0-7
        green: 0-7
        blue: 0-7
        ramp_time: 0-7, corresponding to 0s, .2s, .5s, .8s, 1s, 2s, 3s, and 4s
        """
        m.send("MNSLC {} {} {} {} {}\n".format(self.led_eyes, red, green, blue, ramp_time))

    def hug(self):
        self.rightarm(120,240)
        self.leftarm(120,240)
        sleep(2)
        self.forward(1)
        sleep(2)
        self.restarms()

    def point(self):
        self.rightarm(120,240)

    def restarms(self):
        self.rightarm(0,120)
        self.leftarm(0,120)

    def wave(self):
        self.rightarm(240,120,1000)
        sleep(1.5)
        self.rightarm(120,120)
        sleep(0.8)
        self.rightarm(240,120)
        sleep(0.8)
        self.rightarm(0,120,1500)


    def command(self, cmd):
        if cmd[0] == 'STOP':
            self.stopbase()
        elif cmd[0] == 'FORWARD':
            if len(cmd)==1:
                self.forward(None)
            else:
                self.forward(float(cmd[1]))
        elif cmd[0] == 'REVERSE':
            if len(cmd)==1:
                self.reverse(None)
            else:
                self.reverse(float(cmd[1]))
        elif cmd[0] == 'TURNRIGHT':
            if len(cmd)==1:
                self.turnright(None)
            else:
                self.turnright(float(cmd[1]))
        elif cmd[0] == 'TURNLEFT':
            if len(cmd)==1:
                self.turnright(None)
            else:
                self.turnright(float(cmd[1]))
        elif cmd[0] == 'POINT':
            m.point()
        elif cmd[0] == 'WAVE':
            m.wave()
        elif cmd[0] == 'RESTARMS':
            m.restarms()
        elif cmd[0] == 'HUG':
            print('Cannot hug until you fix the power source for my servos :(')
            #m.hug()
        else:
            print('Unrecognized command {}'.format(cmd[0]))


m = MECCMotion()
m.initialize()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'meccmotion heard %s', data.data)
    cmd = str(data.data).split(' ')
    print(str(cmd))
    m.command(cmd)


def motion_listener():

    rospy.init_node('motion_listener', anonymous=True)

    rospy.Subscriber('motion', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    motion_listener()
