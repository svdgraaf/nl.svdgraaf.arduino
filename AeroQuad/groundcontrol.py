#!/usr/bin/env python 

import serial
import time

s = serial.Serial('/dev/tty.SLAB_USBtoUART', 19200)
print s
# s = serial.Serial('/dev/tty')
# s = serial.Serial('/dev/tty.usbmodemfa141')
# s = serial.Serial('/dev/tty.usbmodemfd131', 115200)

while 1:
  readLine = s.readline()
  print readLine
  # print "."