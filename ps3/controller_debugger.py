#!/usr/bin/env python
import pygame
from pygame.locals import *
import usb.core
import time
 
# init joystick
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()

# s = serial.Serial('/dev/tty.SLAB_USBtoUART')
# s = serial.Serial('/dev/tty.usbmodemfa141')
# s = serial.Serial('/dev/tty.usbmodemfd131')

print 'Initialized Joystick : %s' % j.get_name()
print 'axis: %s, buttons: %s' % (j.get_numaxes(), j.get_numbuttons())

# threshold for (analog) axis
threshold = 0.1

# left
axis0 = 0
axis1 = 0
left = None

# right
axis2 = 0
axis3 = 0
right = None

try:
    while True:
        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                if abs(event.value) > threshold:
                    if event.axis == 0:
                        axis0 = event.value
                    if event.axis == 1:
                        axis1 = event.value
                    if event.axis == 2:
                        axis2 = event.value
                    if event.axis == 3:
                        axis3 = event.value
                    
                    if axis1 < 0 and axis0 < 0:
                        # topleft
                        left = 270 + int(90 * axis1 * -1)
                    elif axis1 > 0 and axis0 < 0:
                        # bottomleft
                        left = 270 - int(90*axis1)
                    elif axis1 > 0 and axis0 > 0:
                        # bottomright
                        left = int(90 * axis1) + 90
                    elif axis1 < 0 and axis0 > 0:
                        # topright
                        left = int(90 * axis1) + 90
                    print left
                    
                        
            if event.type == JOYBUTTONUP:
                print event
            if event.type == JOYBUTTONDOWN:
                print event
except KeyboardInterrupt:
    j.quit()
