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


class MECControl():
    def __init__(self, device='/dev/ttyUSB0', baud=9600,
                 rarm_pin=13, rarm_nserv=2, larm_pin=11, larm_nserv=2):
        self.serial = None
        self.device = device
        self.baud = baud
        self.rarm = rarm_pin
        self.larm = larm_pin
        self.lmotor = [3, 4, 5]
        self.rmotor = [6, 7, 8]

    def send(self, command):
        self.serial.write(command)
        sleep(0)
        response = self.serial.readline()
        print(response)

    def initialize(self):
        self.serial = serial.Serial(self.device, self.baud)
        sleep(4)
        self.send('V\n')
        self.send('MNI {}\n'.format(self.rarm))
        self.send('MNI {}\n'.format(self.larm))
        self.send('DPM {} O\n'.format(self.lmotor[1]))
        self.send('DPM {} O\n'.format(self.lmotor[2]))
        self.send('DPM {} O\n'.format(self.rmotor[1]))
        self.send('DPM {} O\n'.format(self.rmotor[2]))

    def forward(self, duration=1):
        self.send('DW {} 1'.format(self.lmotor[2]))
        self.send('DW {} 1'.format(self.rmotor[2]))
        self.send('AW {} 255 2000'.format(self.lmotor[0]))
        self.send('AW {} 255 500'.format(self.rmotor[0]))
        sleep(duration)
        self.send('AW {} 0 2000'.format(self.lmotor[0]))
        self.send('AW {} 0 500'.format(self.rmotor[0]))
        self.send('DW {} 0'.format(self.lmotor[2]))
        self.send('DW {} 0'.format(self.rmotor[2]))

    def reverse(self, duration=1):
        self.send('DW {} 1'.format(self.lmotor[1]))
        self.send('DW {} 1'.format(self.rmotor[1]))
        self.send('AW {} 255 2000'.format(self.lmotor[0]))
        self.send('AW {} 255 500'.format(self.rmotor[0]))
        sleep(duration)
        self.send('AW {} 0 2000'.format(self.lmotor[0]))
        self.send('AW {} 0 500'.format(self.rmotor[0]))
        self.send('DW {} 0'.format(self.lmotor[1]))
        self.send('DW {} 0'.format(self.rmotor[1]))

    def turnright(self, angle=90):
        self.send('DW {} 1'.format(self.lmotor[2]))
        self.send('AW {} 255 0'.format(self.lmotor[0]))
        sleep(angle/90.0)
        self.send('AW {} 0 0'.format(self.lmotor[0]))
        self.send('DW {} 0'.format(self.lmotor[2]))

    def turnleft(self, angle=90):
        self.send('DW {} 1'.format(self.rmotor[2]))
        self.send('AW {} 255 0'.format(self.rmotor[0]))
        sleep(angle/90.0)
        self.send('AW {} 0 0'.format(self.rmotor[0]))
        self.send('DW {} 0'.format(self.rmotor[2]))

    def move_arm(self, rarm=True, shoulder_pos=90, elbow_pos=90):
        pin = self.rarm
        if not rarm:
            pin = self.larm
            shoulder_pos = 240 - shoulder_pos
            elbow_pos = 240 - elbow_pos
        self.send('MNSSMP {0} {1} {2} {3}'.format(pin, 0,
                                                  shoulder_pos, 250))

        self.send('MNSSMP {0} {1} {2} {3}'.format(pin, 1,
                                                  elbow_pos, 250))


m = MECControl()
m.initialize()
