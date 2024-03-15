################################################################################################
# @File DroneKitPX4.py
# Example usage of DroneKit with PX4
#
# @author Sander Smeets <sander@droneslab.com>
#
# Code partly based on DroneKit (c) Copyright 2015-2016, 3D Robotics.
################################################################################################

# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math, thread
from TrackerTimer import TrackTimer
from matplotlib import pyplot as plt 
from drawnow import *
from Optitrack import Optitrack

################################################################################################
# Settings
################################################################################################
op = Optitrack()

connection_string       = 'udp:192.168.4.2:14550'
# connection_string       = 'COM5'
MAV_MODE_AUTO   = 4
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py


# Parse connection argument
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect


################################################################################################
# Init
################################################################################################

# Connect to the Vehicle
print "Connecting"
# vehicle = connect(connection_string, wait_ready=True, baud=57600)
vehicle = connect(connection_string, wait_ready=['system_status','mode'])
track_time = TrackTimer(vehicle)
track_time.debug = True

################################################################################################
# Listeners
################################################################################################

time_unix_usec = 0

#Create a message listener for home position fix
# @vehicle.on_message('HIGHRES_IMU')
# def listener(self, name, message):
    # time_boot_ms = message.time_usec

# @vehicle.on_message("TIMESYNC")
# def listener_timesync(subself, name, message):
# 	time_unix_usec = message.ts1 / 1000
	# print time_unix_usec
# @vehicle.on_message("LOCAL_POSITION_NED")
# def listener_timesync(subself, name, message):
# 	print message.x, message.y
	# values.append(message.vx)
	# values.pop(0)
	# drawnow(plotValues)

################################################################################################
# Plot function
################################################################################################
values = []

plt.ion()
cnt=0

def plotValues():
    plt.title('Serial value from Arduino')
    plt.grid(True)
    plt.ylabel('Values')
    plt.plot(values, 'rx-', label='values')
    plt.legend(loc='upper right')

#pre-load dummy data
for i in range(0,100):
    values.append(0)

################################################################################################
# Start mission example
################################################################################################

# Display basic vehicle state
print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
print " GPS: %s" % vehicle.gps_0
print " Alt: %s" % vehicle.location.global_relative_frame.alt

while True:
	# track_time.update()
	# msg = vehicle.message_factory.att_pos_mocap_encode(time_boot_ms,[1,0,0,0],5,0,0)
	msg = vehicle.message_factory.att_pos_mocap_encode(int(time.time() * 1e+6),
														op.orientation,
														op.position[0],
														op.position[1],
														op.position[2])
	# print op.position
	vehicle.send_mavlink(msg)
	# print " POS: %s" % vehicle.location.local_frame
	# counter += 0.0001
	# print track_time.estimate()
	# print 'a'
	time.sleep(.02)

# Load commands
cmds = vehicle.commands
cmds.clear()

time.sleep(1)

# Close vehicle object before exiting script
vehicle.close()
time.sleep(1)