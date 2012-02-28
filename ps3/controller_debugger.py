#!/usr/bin/env python
import pygame
from pygame.locals import *
import usb.core
import time
 
# init joystick
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()

print 'Initialized Joystick : %s' % j.get_name()
print 'axis: %s, buttons: %s' % (j.get_numaxes(), j.get_numbuttons())

# threshold for (analog) axis
threshold = 0.1

try:
    while True:
        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                if abs(event.value) > threshold:
                    if event.axis == 0:
                        print event
                        
            if event.type == JOYBUTTONUP:
                print event
            if event.type == JOYBUTTONDOWN:
                print event
except KeyboardInterrupt:
    j.quit()
