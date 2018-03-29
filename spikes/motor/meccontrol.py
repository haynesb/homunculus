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
    def __init__(self, device='/dev/ttyUSB0', baud=57600,
                 rarm_pin=13, rarm_nserv=2, larm_pin=11, larm_nserv=2):
        self.serial = None
        self.device = device
        self.baud = baud
        self.rarm = rarm_pin
        self.larm = larm_pin
        self.lmotor = [3, 4, 5]
        self.rmotor = [6, 7, 8]

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
        self.send('MNI {}\n'.format(self.rarm))
        self.send('MNI {}\n'.format(self.larm))
        self.send('DPM {} O\n'.format(self.lmotor[1]))
        self.send('DPM {} O\n'.format(self.lmotor[2]))
        self.send('DPM {} O\n'.format(self.rmotor[1]))
        self.send('DPM {} O\n'.format(self.rmotor[2]))

    def forward(self, duration=1):
        command = 'DW {} 1\n'.format(self.lmotor[2])
        command += 'DW {} 1\n'.format(self.rmotor[2])
        command += 'AW {} 200 1\n'.format(self.lmotor[0])
        command += 'AW {} 200 1\n'.format(self.rmotor[0])
        self.send(command)
        sleep(duration)
        command += 'AW {} 0 1\n'.format(self.lmotor[0])
        command += 'AW {} 0 1\n'.format(self.rmotor[0])
        command = 'DW {} 0\n'.format(self.lmotor[2])
        command += 'DW {} 0\n'.format(self.rmotor[2])
        self.send(command)

    def reverse(self, duration=1):
        command = 'DW {} 1\n'.format(self.lmotor[1])
        command += 'DW {} 1\n'.format(self.rmotor[1])
        command += 'AW {} 200 1\n'.format(self.lmotor[0])
        command += 'AW {} 200 1\n'.format(self.rmotor[0])
        self.send(command)
        sleep(duration)
        command += 'AW {} 0 1\n'.format(self.lmotor[0])
        command += 'AW {} 0 1\n'.format(self.rmotor[0])
        command = 'DW {} 0\n'.format(self.lmotor[1])
        command += 'DW {} 0\n'.format(self.rmotor[1])
        self.send(command)

    def turnright(self, angle=90):
        command = 'DW {} 1\n'.format(self.lmotor[2])
        command += 'DW {} 1\n'.format(self.rmotor[1])
        command += 'AW {} 150 1\n'.format(self.lmotor[0])
        command += 'AW {} 150 1\n'.format(self.rmotor[0])
        self.send(command)
        sleep(angle/90.0)
        command += 'AW {} 0 1\n'.format(self.lmotor[0])
        command += 'AW {} 0 1\n'.format(self.rmotor[0])
        command = 'DW {} 0\n'.format(self.lmotor[2])
        command += 'DW {} 0\n'.format(self.rmotor[1])
        self.send(command)




    def turnleft(self, angle=90):
        command = 'DW {} 1\n'.format(self.lmotor[1])
        command += 'DW {} 1\n'.format(self.rmotor[2])
        command += 'AW {} 150 1\n'.format(self.lmotor[0])
        command += 'AW {} 150 1\n'.format(self.rmotor[0])
        self.send(command)
        sleep(angle/90.0)
        command += 'AW {} 0 1\n'.format(self.lmotor[0])
        command += 'AW {} 0 1\n'.format(self.rmotor[0])
        command = 'DW {} 0\n'.format(self.lmotor[1])
        command += 'DW {} 0\n'.format(self.rmotor[2])
        self.send(command)

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

