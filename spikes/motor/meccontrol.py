import serial
from time import sleep


class MECControl():
    def __init__(self, device='/dev/ttyUSB0', baud=9600,
                 rarm_pin=13, rarm_nserv=2, larm_pin=11, larm_nserv=2):
        self.serial = None
        self.device = device
        self.baud = baud
        self.rarm = rarm_pin
        self.larm = larm_pin

    def send(self, command):
        self.serial.write(command)
        sleep(0.15)
        response = self.serial.readline()
        print(response)

    def initialize(self):
        self.serial = serial.Serial(self.device, self.baud)
        sleep(4)
        self.send('V\n')
        self.send('MNI {}\n'.format(self.rarm))
        self.send('MNI {}\n'.format(self.larm))

    def move_arm(self, rarm=True, shoulder_pos=90, elbow_pos=90):
        pin = self.rarm
        if not rarm:
            pin = self.larm
            shoulder_pos = 240 - shoulder_pos
            elbow_pos = 240 - elbow_pos
        self.send('MNSSMP {0} {1} {2} {3}'.format(pin, 0,
                                                  shoulder_pos, 2000))

        self.send('MNSSMP {0} {1} {2} {3}'.format(pin, 1,
                                                  elbow_pos, 2000))
