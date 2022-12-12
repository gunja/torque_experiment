#!/usr/bin/env python2

import rospy
import roslib
import serial
import os
from armin_robothw.msg import BatteryState
from std_msgs.msg import Bool, String

power_off_requested = False
management_requested_apps = {}

# TODO make reading from rosparams later
MaxBatteryLevel = 76
MinBatteryLevel = 63

def callback(data):
    global power_off_requested
    if data.data == True:
        power_off_requested = True
        print "Power Off Request received"

def listManagement(data):
    global management_requested_apps
    if data.data[:4] == "Done":
        name = data.data[5:]
        management_requested_apps.pop(name, "super")
    else:
        management_requested_apps[data.data] = "registered"

def make_shutdown_sequence(serPort):
    msgOff = b'S01R0001\r'
    serPort.write(msgOff)
    r = serPort.readall()
    print 'r=', r
    os.system('sudo shutdown -h +0')

def main():
    rospy.init_node('battery_manager', anonymous=False)
    serPort = serial.Serial(port = '/dev/ttyS0', baudrate=2400, timeout = 1)
    rate = rospy.Rate(0.2) # each 5 seconds
    pub = rospy.Publisher('battery_state', BatteryState, queue_size=1)
    sbs = rospy.Subscriber('power_cmd', Bool, callback)

    sbsListing = rospy.Subscriber('/battery/dependecie', String, listManagement) 
    pubBatteryCmds = rospy.Publisher('/battery/commands', String,  queue_size=10)

    msgOff = b'S01R0001\r'
    msgGetState = b'Q1\r'

    while not rospy.is_shutdown():
        serPort.write(msgGetState)
        lv = b''
        tot = b''
        while lv != b'\r':
            lv = serPort.read()
            tot = tot + lv

        print 'reply from bat:', tot
        bs = BatteryState()
        parts = tot.split()
        flags = int( parts[7], 2)
        bs.on_battery = (flags & 0x80)
        bs.percentage = int(100 *(float(parts[5]) - MinBatteryLevel) / (MaxBatteryLevel - MinBatteryLevel))
        pub.publish(bs)

        if power_off_requested:
            pubBatteryCmds.publish( String(data = "prepareShutdown"))
            if len(management_requested_apps) == 0 and (bs.on_battery > 0):
                make_shutdown_sequence(serPort)

        elif bs.percentage < 10:
            make_shutdown_sequence(serPort)

        rate.sleep()

if __name__ == '__main__':
    main()
