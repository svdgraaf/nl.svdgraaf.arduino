#!/usr/bin/env python

import time
from datetime import datetime
import serial

print
print "================================================"
print "        MultiWii Joystick Serial Radio"
print "================================================"

try:
    import pygame.joystick
except ImportError:
    print
    print "Please install the 'pygame' module <http://www.pygame.org/>."
    print
    quit()

# XBee Node Name (this improves reliability)
XBEE_DESTINATION_NODE = None

BAUD_RATE = 19200
# SERIAL_PORT = '/dev/tty.usbserial-A70064Mh'
# SERIAL_PORT = '/dev/tty.usbmodemfa141'
# SERIAL_PORT = '/dev/tty.usbmodemfd131'
# SERIAL_PORT = '/dev/tty.usbmodemfa141321'
SERIAL_PORT = '/dev/cu.SLAB_USBtoUART'
# SERIAL_PORT = '/dev/tty.usbserial-AH00PP05'

# 50hz
FREQUENCY = 1. / 100

MIN_PPM = 0
MAX_PPM = 250

# AXIS_ROLL = 2
# AXIS_PITCH = 3
# AXIS_THROTTLE = 1
# AXIS_YAW = 0

AXIS_ROLL = 0
AXIS_PITCH = 1
AXIS_THROTTLE = 3
AXIS_YAW = 2

BUTTON_AUX1 = 0 # Trigger
BUTTON_AUX2 = 1 # Button 2
BUTTON_AUX3 = 2 # Button 3
BUTTON_AUX4 = 3 # Button 4

class ControllerState(object):
    throttle = MIN_PPM
    roll     = MIN_PPM
    pitch    = MIN_PPM
    yaw      = MIN_PPM
    aux1     = MAX_PPM
    aux2     = MIN_PPM
    aux3     = MIN_PPM
    aux4     = MIN_PPM
    armed    = False
    
    def __str__(self):
        return 'Throttle:{0.throttle} Roll:{0.roll} Pitch:{0.pitch} Yaw:{0.yaw} A1:{0.aux1}'.format(self)

    def serial_format(self):
        #return 'Q' + chr(self.throttle) + chr(self.roll) + chr(self.pitch) + chr(self.yaw) + chr(self.aux1) + \
        #    chr(self.aux2) + chr(self.aux3) + chr(self.aux4)
        #return 'Q' + chr(self.throttle) + chr(self.roll) + chr(self.pitch) + chr(self.yaw) + chr(self.aux1)
        return chr(self.throttle) + chr(self.roll) + chr(self.pitch) + chr(self.yaw) + chr(self.aux1) + \
            chr(self.aux2) + chr(self.aux3) + chr(self.aux4) + chr(254)

    # Joystick Movement
    def handleJoyEvent(self, e):
        # Identify joystick axes and assign events
        if e.type == pygame.JOYAXISMOTION:
            axis = e.dict['axis']
            # Convert -1.0 - +1.0 to 0 - 255
            if True: #e.dict['value'] < -0.1 or e.dict['value'] > 0.1:
              value = int(e.dict['value'] * (MAX_PPM / 2)) + (MAX_PPM / 2)

              # print 'Axis: {}, value: {}'.format(axis, value)

              if axis == AXIS_ROLL:
                  self.roll = value
              elif axis == AXIS_PITCH:
                  self.pitch = MAX_PPM - value
              elif axis == AXIS_THROTTLE:
                  self.throttle = MAX_PPM - value
              elif axis == AXIS_YAW:
                  self.yaw = value

        # Button Presses (toggle)
        elif e.type == pygame.JOYBUTTONDOWN:
            button = e.dict['button']

            print 'Button Down: {}'.format(button)
            
            if button == BUTTON_AUX1:
                self.aux1 = MAX_PPM if self.aux1 == MIN_PPM else MIN_PPM
                self.armed = True
            elif button == BUTTON_AUX2:
                self.aux2 = MAX_PPM if self.aux2 == MIN_PPM else MIN_PPM
            elif button == BUTTON_AUX3:
                self.aux3 = MAX_PPM if self.aux3 == MIN_PPM else MIN_PPM
            elif button == BUTTON_AUX4:
                self.aux4 = MAX_PPM if self.aux4 == MIN_PPM else MIN_PPM

        # # Assign actions for Coolie Hat Switch events
        # elif e.type == pygame.JOYHATMOTION:
        #     if (e.dict['value'][0] == -1):
        #         print "Hat Left"
        #         servo.move(4, 0)
        #     if (e.dict['value'][0] == 1):
        #         print "Hat Right"
        #         servo.move(4, 180)
        #     if (e.dict['value'][1] == -1):
        #         print "Hat Down"
        #     if (e.dict['value'][1] == 1):
        #         print "Hat Up"
        #     if (e.dict['value'][0] == 0 and e.dict['value'][1] == 0):
        #         print "Hat Centered"
        #         servo.move(4, 90)

        # print self

# Main method
def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0) as xbee:
        time.sleep(5)
        xbee.write('0')
        time.sleep(1)
      
        controller_state = ControllerState()
        watchdog_timer = 0
        joysticks = []

        # Initialize pygame
        pygame.joystick.init()
        pygame.display.init()

        if not pygame.joystick.get_count():
            print "Please connect a joystick and run again."
            print
            quit()

        print "%s joystick(s) detected." % pygame.joystick.get_count()

        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            joysticks.append(joystick)
            print "Joystick %d: " % (i) + joysticks[i].get_name()

        # Set destination node
        if XBEE_DESTINATION_NODE:
            print 'Setting XBee destination node to %s' % XBEE_DESTINATION_NODE
            xbee.write('+++')
            time.sleep(.5)
            xbee.write('ATDN')
            xbee.write(XBEE_DESTINATION_NODE)
            xbee.write('\r')
            time.sleep(1)
            print 'Destination node set'

        # Run joystick listener loop
        while True:
            poll_started = datetime.now()

            while True:
                # Dequeue all joystick events and update state
                e = pygame.event.poll()
                # print e
                if e.type in (pygame.JOYAXISMOTION, pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP, pygame.JOYHATMOTION):
                    controller_state.handleJoyEvent(e)
                if e.type == pygame.NOEVENT:
                    break

            #s = xbee.read(1000000)

            #if s:
            #    print 'Incoming Serial:', s

            # Sleep only long enough to keep the output at 50Hz
            poll_ended = datetime.now()
            sleep_time = FREQUENCY - ((poll_ended - poll_started).microseconds / 1000000.)
            
            if sleep_time > 0.:
                time.sleep(sleep_time)
                # print controller_state
                # print controller_state.armed
                xbee.write(controller_state.serial_format())
                # xbee.write('s')
                # time.sleep(1)
                # print xbee.readline()
                # print xbee.readline()
              
            write_ended = datetime.now()

            watchdog_timer += (write_ended - poll_started).microseconds

            # Print out the state every once in a while to make sure the program hasn't died
            if watchdog_timer > 5 * 100000:
                print controller_state
                # xbee.write('s')
                # xbee.write('x')
                # time.sleep(1)
                watchdog_timer = 0

            #print 'Loop Time:', (write_ended - poll_started).microseconds

# Allow use as a module or standalone script
if __name__ == "__main__":
    main()