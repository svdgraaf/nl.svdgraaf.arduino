#!/usr/bin/env python 

import pygame
from pygame.locals import *
import serial
import time

# init pygame stuff
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()

# s = serial.Serial('/dev/tty.SLAB_USBtoUART')
# s = serial.Serial('/dev/tty.usbmodemfa141')
s = serial.Serial('/dev/tty.usbmodemfd131')
pygame.init()

# start by sending full stop
current = "[0|0|0|]"
s.write(current)

print 'Initialized Joystick : %s' % j.get_name()
print 'axis: %s, buttons: %s, hats: %s' % (j.get_numaxes(), j.get_numbuttons(), j.get_numhats())

while True:
  # get the pygame queue filled
  pygame.event.pump()

  # check left joystick
  if j.get_axis(0) < -0.5:
    a = 0
    d = 1
  elif j.get_axis(0) > 0.5:
    a = 1
    d = 0
  else:
    a = 0
    d = 0
    
  # combine into command
  cmd = "[%s|%s|%s|%s]" % (j.get_button(9),a, j.get_button(8), d)

  # only send the command over serial, if it's different
  if cmd != current:
    print cmd
    current = cmd
    s.write(cmd)