#!/usr/bin/env python 

import pygame
from pygame.locals import *
import serial

s = serial.Serial('/dev/tty.SLAB_USBtoUART')
# s = serial.Serial('/dev/tty.usbmodemfa141')
# s = serial.Serial('/dev/tty.usbmodemfd131')
pygame.init()

done = False
while not done:
    for event in pygame.event.get():
        if (event.type == KEYUP) or (event.type == KEYDOWN):
            print event
            if (event.key == K_ESCAPE):
                done = True
                
        if(event.type == KEYDOWN):
            # f.write(event.unicode)
            s.write(event.unicode)
            
        if(event.type == KEYUP):
            s.write('@')
            s.write(chr(event.key))
            pass
