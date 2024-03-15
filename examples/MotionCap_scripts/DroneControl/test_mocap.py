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


op = OptiTrack()
while True:
	print(op.position)

