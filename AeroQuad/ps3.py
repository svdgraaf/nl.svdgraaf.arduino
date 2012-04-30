#!/usr/bin/env python 

import pygame
from pygame.locals import *
import serial
import time

# init pygame stuff
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()

MIN_PPM = 0
MAX_PPM = 250

AXIS_YAW = 0
AXIS_THROTTLE = 1
AXIS_ROLL = 2
AXIS_PITCH = 3

throttle = MIN_PPM
roll     = MIN_PPM
pitch    = MIN_PPM
yaw      = MIN_PPM

# extra buttons
aux1     = MIN_PPM
aux2     = MIN_PPM
aux3     = MIN_PPM
aux4     = MIN_PPM

s = serial.Serial('/dev/tty.SLAB_USBtoUART')
# s = serial.Serial('/dev/tty')
# s = serial.Serial('/dev/tty.usbmodemfa141')
# s = serial.Serial('/dev/tty.usbmodemfd131')
pygame.init()

def build_cmd():
  cmd = chr(throttle) + chr(roll) + chr(pitch) + chr(yaw) + chr(aux1) + chr(aux2) + chr(aux3) + chr(aux4) + chr(254)  
  return cmd

print 'Initialized Joystick : %s' % j.get_name()
print 'axis: %s, buttons: %s, hats: %s' % (j.get_numaxes(), j.get_numbuttons(), j.get_numhats())

while True:
  # get the pygame queue filled
  pygame.event.pump()
  e = pygame.event.poll()

  # we check for new commands whenever there is an action on the controller
  if e.type in (pygame.JOYAXISMOTION, pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP, pygame.JOYHATMOTION):

    # PS3 controllers return scale from -1 to +1, we need to 
    # recalculate to MIN_PPM - MAX_PPM

    # check left horizontal stick (yaw)
    if j.get_axis(AXIS_YAW) < -0.1 or j.get_axis(AXIS_YAW) > 0.1:
      yaw = int((j.get_axis(AXIS_YAW) + 1) * (MAX_PPM / 2))
    else:
      yaw = (MAX_PPM / 2)

    # check left vertical (throttle)
    if j.get_axis(AXIS_THROTTLE) < -0.5 or j.get_axis(AXIS_THROTTLE) > 0.5:
      throttle = MAX_PPM - int((j.get_axis(AXIS_THROTTLE) + 1) * (MAX_PPM / 2))
    else:
      throttle = (MAX_PPM / 2)

    # check right horizontal (roll)
    if j.get_axis(AXIS_ROLL) < -0.5 or j.get_axis(AXIS_ROLL) > 0.5:
      roll = int((j.get_axis(AXIS_ROLL) + 1) * (MAX_PPM / 2))
    else:
      roll = (MAX_PPM / 2)

    # check right vertical (pitch)
    if j.get_axis(AXIS_PITCH) < -0.5 or j.get_axis(AXIS_PITCH) > 0.5:
      pitch = MAX_PPM - int((j.get_axis(AXIS_PITCH) + 1) * (MAX_PPM / 2))
    else:
      pitch = (MAX_PPM / 2)

    cmd = build_cmd()

    # only send the command over serial, if it's different from the previous
    if cmd != current:
      print "%s, %s, %s, %s, %s, %s, %s, %s " % (throttle, roll, pitch, yaw, aux1, aux2, aux3, aux4)
      current = cmd
      s.write(cmd)