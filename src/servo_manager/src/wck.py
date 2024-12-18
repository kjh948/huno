#!/usr/bin/python
# -*- coding: utf-8 -*-

# ** wck.py **
# Copyright (C) 2013 Prashanta Shrestha

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import sys
import serial
import struct
import time
import array


class servo:

    port = '/dev/ttyUSB0'
    baud = 115200
    HEADER = 255
    sid = 0x00
    ser = 0x00
    baudrates = [115200]
        

    def __init__(
        self,
        port,
        baud
        ):
        self.data = []
        if port:
            self.port = port
        if baud:
            self.baud = baud
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        
    def connect(
        self,
        port,
        baud,
        t,
        ):
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(self.port, self.baud, timeout=t)

    # SCAN FOR SERVO ID AND BAUDRATE

    def scan(self):
        foundbaud = -1
        foundid = -1
        for w in self.baudrates:
            if foundid < 0x00:
                print ('Scanning at %d' % w)
                self.connect(self.port, w, 0.2)
                for x in range(31):
                    self._sendSetCmd(7 << 5 | x, 0x0C, x, x)
                    time.sleep(0.5)
                    data = self.ser.read(1)
                    if data:
                        data_unpacked = struct.unpack('>B', data)
                        if x == data_unpacked[0x00]:
                            foundid = x
                            foundbaud = w
                            print ('FOUND!')
                            break
                    elif x == 30:
                        print ('Not found')
            else:
                print ('Baudrate : %d' % foundbaud)
                print ('ID : %d' % foundid)
                break
        self.connect(self.port, self.baud, 3)

    # SEND SET COMMAND TO SERVO........

    def _sendSetCmd(
        self,
        data1,
        data2,
        data3,
        data4,
        ):
        chksum = (data1 ^ data2 ^ data3 ^ data4) & 127
        self.ser.flushInput()

        # print "SENDING:" + hex(self.HEADER) + " " + hex(data1) + " " + hex(data2) + " " + hex(data3) + " " + hex(data4) + " " + hex(chksum)........

        self.ser.write(struct.pack('<B', self.HEADER))
        self.ser.write(struct.pack('<B', data1))
        self.ser.write(struct.pack('<B', data2))
        self.ser.write(struct.pack('<B', data3))
        self.ser.write(struct.pack('<B', data4))
        self.ser.write(struct.pack('<B', chksum))

    # SEND COMMAND TO SERVO

    def _sendCmd(self, data1, data2):
        chksum = (data1 ^ data2) & 127
        self.ser.flushInput()

        # print "SENDING:" + hex(self.HEADER) + " " + hex(data1) + " " + hex(data2) + " " + hex(chksum)........

        self.ser.write(struct.pack('<B', self.HEADER))
        self.ser.write(struct.pack('<B', data1))
        self.ser.write(struct.pack('<B', data2))
        self.ser.write(struct.pack('<B', chksum))

    # READ ROUTINE

    def _read(self, f1, f2):
        time.sleep(0.5)
        data = self.ser.read(1)
        if data:
            data_unpacked = struct.unpack('>B', data)
            print (f1 + ': 0x%s (%d)' % (data.hex(),
                    data_unpacked[0x00]))
        data = self.ser.read(1)
        if data:
            data_unpacked = struct.unpack('>B', data)
            print (f2 + ': 0x%s (%d)' % (data.hex(),
                    data_unpacked[0x00]))
            return data_unpacked[0]
        return -1

    # READ STATUS

    def readStatus(self, id):
        self._sendCmd(5 << 5 | id, 0x00)
        return self._read('Load', 'Position')

    # READ CURRENT SERVO POSITION

    def readPos(self, id):
        self._sendCmd(0xa0 | id, 0x00)
        return self._read('Load', 'Position')


    # MOVE SERVO TO SPECIFIED TARGET POSITION (1-254)

    def pos(self, id, torque, target):
        self._sendCmd(torque << 5 | id, target)
        return self._read('Load', 'Position')

    # MOVE SERVO TO SPECIFIED TARGET POSITION (1-254)

    def posGroup(self, id, torque, target):
        
        self._sendCmd(torque << 5 | id, target)
        return self._read('Load', 'Position')
    
    # ROTATE WHEEL IN CLOCKWISE DIRECTION AT GIVEN SPEED (1-15, 0: stop)

    def cw(self, id, speed):
        self._sendCmd(6 << 5 | id, 4 << 4 | speed)
        return self._read('No. of Rotation', 'Position')

    # ROTATE WHEEL IN COUNTER-CLOCKWISE DIRECTION AT GIVEN SPEED (1-15, 0: stop)....

    def ccw(self, id, speed):
        self._sendCmd(6 << 5 | id, 3 << 4 | speed)
        self._read('No. of Rotation', 'Position')

    # SET SERVO ID

    def setID(self, id, newid):
        self._sendSetCmd(7 << 5 | id, 0x0C, newid, newid)
        return self._read('New ID', 'New ID')

    # READ SPEED

    def readSpeed(self, id):
        self._sendSetCmd(7 << 5 | id, 0x0E, 0x00, 0x00)
        return self._read('Speed', 'Acceleration')

    # SET SPEED (0-30) and ACCELERATION (20-100)

    def setSpeed(self, id, speed, accel):
        self._sendSetCmd(7 << 5 | id, 0x0D, speed, accel)
        return self._read('Speed', 'Acceleration')

    def readPD(self, id):
        self._sendSetCmd(7 << 5 | id, 0x0A, 0x00, 0x00)
        return self._read('P-Gain', 'D-Gain')

    def readI(self, id):
        self._sendSetCmd(7 << 5 | id, 0x16, 0x00, 0x00)
        return self._read('I-Gain', 'I-Gain')

    def passivate(self, id):
        self._sendCmd(6 << 5 | id, 1 << 4 | 0x00)
        return self._read('Data2', 'Position')

    # CLOSE SERIAL PORT

    def close(self):
        self.ser.close()


if __name__=="__main__":
    a = servo("/dev/ttyUSB0",115200)

    a.scan()
    id = 14
    curpos = a.readPos(id)

    print("current pos", curpos)

    delta = 30
    a.pos(id, 4,curpos+delta)
    time.sleep(1)
    a.pos(id, 4,curpos-delta)
    time.sleep(1)

    a.pos(id, 4,curpos)
    time.sleep(1)
    a.close()