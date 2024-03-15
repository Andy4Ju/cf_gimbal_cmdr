################################################################################################
# @File DroneKitPX4.py
# Example usage of DroneKit with PX4
#
# coding: utf8
# @author Sander Smeets <sander@droneslab.com>
#
# Code partly based on DroneKit (c) Copyright 2015-2016, 3D Robotics.
################################################################################################

# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math
from TrackerTimer import TrackTimer
from Optitrack import OptiTrack
import threading

vehicle_mode = 'normal' # 'normal', 'trajectory'

################################################################################################
# Settings
################################################################################################
# connection_string       = 'udp:192.168.1.137:14558'
connection_string       = 'udp:192.168.0.102:14558'
# connection_string       = 'COM5'
MAV_MODE_AUTO   = 4
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py00

################################################################################################
# Init
################################################################################################

# Connect to the Vehicle
print("Connecting")
print("In %s mode"%vehicle_mode)
# vehicle = connect(connection_string, wait_ready=True, baud=57600)
vehicle = connect(connection_string, wait_ready=['system_status','mode','parameters'])
track_time = TrackTimer(vehicle)
track_time.debug = True


################################################################################################
# Threads
################################################################################################

def local_position_threads():
	# Start motion capture streaming
	op = OptiTrack()

	while True:
		# msg = vehicle.message_factory.att_pos_mocap_encode(int(time.clock() * 1e+6),
		# 														[1,0,0,0],
		# 														5,
		# 														1,
		# 														0)
		if vehicle_mode is 'trajectory':
			omega = 0.3 * time.clock()
			traj_x = 0.5 * (math.cos(2*omega) * math.cos(omega))
			traj_y = 0.5 * (math.cos(2*omega) * math.sin(omega))
			msg = vehicle.message_factory.att_pos_mocap_encode(int(time.clock() * 1e+6),
																op.rotation,
																traj_x + op.position[0],
																traj_y + op.position[1],
																op.position[2])
		else:
			msg = vehicle.message_factory.att_pos_mocap_encode(int(time.clock() * 1e+6),
																op.rotation,
																op.position[0],
																op.position[1],
																op.position[2])
			# print(op.position)
		# print op.position
		vehicle.send_mavlink(msg)
		vehicle.flush()
		time.sleep(.005)


def set_position_target():
	while True:
		msg1 = vehicle.message_factory.set_position_target_local_ned_encode(
		        0,       # time_boot_ms (not used)
		        0, 0,    # target system, target component
		        mavutil.mavlink.MAV_FRAME_LOCAL_NED,   # frame
		        0b0000111111111111, # type_mask (only positions enabled)
		        0, 0, 0, # x, y, z positions (used)
		        0, 0, 0, # x, y, z velocity in m/s (not used)
		        0, 0, -1.5, # x, y, z acceleration (not used)
		        0, 0)    # yaw, yaw_rate (not used)
		vehicle.send_mavlink(msg1)
		vehicle.flush()
		time.sleep(.005) 

def timesync_threads():
	while True:
		track_time.update()
		time.sleep(0.01)


################################################################################################
# Start mission example
################################################################################################

# Display basic vehicle state????????????????????????????][] 
print(" Type: %s" % vehicle._vehicle_type)
print(" Armed: %s" % vehicle.armed)
print(" System status: %s" % vehicle.system_status.state)
print(" Alt: %s" % vehicle.location.global_relative_frame.alt)
# vehicle.parameters['ML_ANV_GAIN'] = 1
# vehicle.parameters['ML_ANG_GAIN'] = 1
# vehicle.parameters['ML_VEL_GAIN'] = 1
# vehicle.parameters['ML_POS_GAIN'] = 1
# vehicle.parameters['ML_ACTION_GAIN'] = 4
# print "ML_ACTION_GAIN Param: %s" % vehicle.parameters['ML_ACTION_GAIN']

# sync log file time
msg = vehicle.message_factory.system_time_encode(time.clock()*1e+6,0)
vehicle.send_mavlink(msg)
vehicle.flush()

op_thread = threading.Thread(target=local_position_threads, name='T1')
timesync_thread = threading.Thread(target=timesync_threads, name='T2')
# setpoint_thread = threading.Thread(target=set_position_target, name='T3')
op_thread.start()
# timesync_thread.start()
# setpoint_thread.start()
# setpoint_thread.join()
# timesync_thread.join()
op_thread.join()
