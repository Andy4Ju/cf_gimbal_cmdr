"""
Single Crazyflie test
"""
import logging
import time
import os
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread
from cflib.crazyflie.log import LogConfig

import cflib
from cflib.crazyflie import Crazyflie

import utils2
import enum

logging.basicConfig(level=logging.ERROR)


class PlotTarget(enum.Enum):
	POS = 1
	RATE = 2
	CH = 3


class OmniAttCF:
	def __init__(self, link_uri, index):
		""" Initialize and run the example with the specified link_uri """

		self._cf = Crazyflie(rw_cache='./cache')

		self._cf.connected.add_callback(self._connected)
		self._cf.disconnected.add_callback(self._disconnected)
		self._cf.connection_failed.add_callback(self._connection_failed)
		self._cf.connection_lost.add_callback(self._connection_lost)

		self._cf.open_link(link_uri)

		print('Connecting to %s' % link_uri)

		self.wn = 30
		self.damp = 3
		self.dampx = 15
		self.wn_pos = 0.25*self.wn
		self.c1 = 7.2

		self.gain_name = ['Kwx',  'Kix', 'Kdx',
						'Kwy', 'Kiy', 'Kdy',
						'KRx', 'KRix', 'KRy','KRiy',
						'Kffx', 'Kffy']
		
		# self.gain_value = [1.2*2*self.dampx*self.wn/self.c1, self.wn*self.wn/self.c1, 0.08*2*self.dampx*self.wn/self.c1,
		self.gain_value = [800, 2000, 15,
							1000, 2500, 3,
							10, 6, 22.7, 22.2, 0.6, 0.8]

		self.plotTarget = PlotTarget.RATE

		self.qw_IMU = 0
		self.qx_IMU = 0
		self.qy_IMU = 0
		self.qz_IMU = 0

		self.wx_ref = 0
		self.wy_ref = 0
		self.wz_ref = 0

		self.wx = 0
		self.wy = 0

		self.eRx = 0
		self.eRy = 0
		self.eRz = 0

		self.eWx = 0
		self.eWy = 0
		self.eWz = 0

		self.Tau_x = 0
		self.Tau_y = 0
		self.Tau_z = 0

		self.rollPArt = 0
		self.pitchPart = 0
		self.yawPart = 0
		self.thrustPart = 0

		self.t_m1 = 0
		self.t_m2 = 0
		self.t_m3 = 0
		self.t_m4 = 0

		self.KRx = 0
		self.KWx = 0

		self.timest = 0

		self.CompQuat = 0
		self.wx_r = 0
		self.wy_r = 0
		self.wz_r = 0
		self.thrust = 0
		
	def _connected(self, link_uri):

		print('Connected to %s' % link_uri)

		# modify PID gains parameters
		
		self._cf.param.add_update_callback(group='sparam_omni', name=None,
											cb=self._param_callback)
		for n in self.gain_name:
			ind = self.gain_name.index(n)
			self._cf.param.set_value('sparam_omni.{}'.format(n), '{}'.format(self.gain_value[ind]))

		# The definition of the logconfig can be made before connecting
		self._lg_battery = LogConfig(name='pm', period_in_ms=1000)
		self._lg_battery.add_variable('pm.vbat','float')

		self._lg_stab = LogConfig(name='sctrl_omni', period_in_ms=10)

		if self.plotTarget == PlotTarget.POS:
			self._lg_stab.add_variable('sctrl_omni.qw_IMU', 'float')
			self._lg_stab.add_variable('sctrl_omni.qx_IMU', 'float')
			self._lg_stab.add_variable('sctrl_omni.qy_IMU', 'float')
			self._lg_stab.add_variable('sctrl_omni.qz_IMU', 'float')
		elif self.plotTarget == PlotTarget.RATE:
			self._lg_stab.add_variable('sctrl_omni.wx_r', 'float')
			self._lg_stab.add_variable('sctrl_omni.wy_r', 'float')
			self._lg_stab.add_variable('sctrl_omni.wx', 'float')
			self._lg_stab.add_variable('sctrl_omni.wy', 'float')
			# self._lg_stab.add_variable('sctrl_omni.KRx', 'float')
			# self._lg_stab.add_variable('sctrl_omni.KWx', 'float')

			# self._lg_stab.add_variable('sctrl_omni.eRx', 'float')
			# self._lg_stab.add_variable('sctrl_omni.eRy', 'float')
			# self._lg_stab.add_variable('sctrl_omni.eRz', 'float')

			# self._lg_stab.add_variable('sctrl_omni.eWx', 'float')
			# self._lg_stab.add_variable('sctrl_omni.eWy', 'float')
			# self._lg_stab.add_variable('sctrl_omni.eWz', 'float')

			# self._lg_stab.add_variable('sctrl_omni.Tau_x', 'float')
			# self._lg_stab.add_variable('sctrl_omni.Tau_y', 'float')
			# self._lg_stab.add_variable('sctrl_omni.Tau_z', 'float')

			# self._lg_stab.add_variable('sctrl_omni.rollPart', 'float')
			# self._lg_stab.add_variable('sctrl_omni.pitchPart', 'float')
			# self._lg_stab.add_variable('sctrl_omni.yawPart', 'float')
			# self._lg_stab.add_variable('sctrl_omni.thrustPart', 'float')

			# self._lg_stab.add_variable('sctrl_omni.t_m1', 'float')
			# self._lg_stab.add_variable('sctrl_omni.t_m2', 'float')
			# self._lg_stab.add_variable('sctrl_omni.t_m3', 'float')
			# self._lg_stab.add_variable('sctrl_omni.t_m4', 'float')

			# Adding the configuration cannot be done until a Crazyflie is
			# connected, since we need to check that the variables we
			# would like to log are in the TOC.
		try:
			self._cf.log.add_config(self._lg_battery)
			self._lg_battery.data_received_cb.add_callback(self._battery_log_data)
			self._lg_battery.start()

			self._cf.log.add_config(self._lg_stab)
			# This callback will receive the data
			self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
			# This callback will be called on errors
			self._lg_stab.error_cb.add_callback(self._stab_log_error)
			# Start the logging
			self._lg_stab.start()

		except KeyError as e:
			print('Could not start log configuration,'
				  '{} not found in TOC'.format(str(e)))
			
		except AttributeError:
			print('Could not add Stabilizer log config, bad configuration.')

	def _param_callback(self, name, value):
		print('Readback: {0}={1}'.format(name, value))

	def _battery_log_data(self, timestamp, data, logconf):
		battery_data = round(data['pm.vbat'], 1)
		print('Battery voltage is: |3.1V(E)| --- %s V --- |4.2V(F)|' % battery_data)
		self._lg_battery.data_received_cb.remove_callback(self._battery_log_data)

	def _stab_log_error(self, logconf, msg):
		"""Callback from the log API when an error occurs"""
		print('Error when logging %s: %s' % (logconf.name, msg))

	def _stab_log_data(self, timestamp, data, logconf):
		"""Callback froma the log API when data arrives"""
		self.timest = timestamp
		if self.plotTarget == PlotTarget.POS:
			self.qw_IMU = data["sctrl_omni.qw_IMU"]
			self.qx_IMU = data["sctrl_omni.qx_IMU"]
			self.qy_IMU = data["sctrl_omni.qy_IMU"]
			self.qz_IMU = data["sctrl_omni.qz_IMU"]
		elif self.plotTarget == PlotTarget.RATE:
			self.wx_ref = data["sctrl_omni.wx_r"]
			self.wy_ref = data["sctrl_omni.wy_r"]
			self.wx = data["sctrl_omni.wx"]
			self.wy = data["sctrl_omni.wy"]

			# self.KRx = data["sctrl_omni.KRx"]
			# self.KWx = data["sctrl_omni.KWx"]
			
			# self.eRx = data["sctrl_omni.eRx"]
			# self.eRy = data["sctrl_omni.eRy"]
			# self.eRz = data["sctrl_omni.eRz"]

			# self.wz_ref = data["sctrl_omni.wz_r"]
			# self.eWx = data["sctrl_omni.eWx"]
			# self.eWy = data["sctrl_omni.eWy"]
			# self.eWz = data["sctrl_omni.eWz"]

			# self.Tau_x = data["sctrl_omni.Tau_x"]
			# self.Tau_y = data["sctrl_omni.Tau_y"]
			# self.Tau_z = data["sctrl_omni.Tau_z"]

			# self.rollPArt = data["sctrl_omni.rollPart"]
			# self.pitchPart = data["sctrl_omni.pitchPart"]
			# self.yawPart = data["sctrl_omni.yawPart"]
			# self.thrustPart = data["sctrl_omni.thrustPart"]

			# self.t_m1 = data["sctrl_omni.t_m1"]
			# self.t_m2 = data["sctrl_omni.t_m2"]
			# self.t_m3 = data["sctrl_omni.t_m3"]
			# self.t_m4 = data["sctrl_omni.t_m4"]

	def _connection_failed(self, link_uri, msg):
		"""Callback when connection initial connection fails (i.e no Crazyflie at the specified address)"""
		print('Connection to %s failed: %s' % (link_uri, msg))

	def _connection_lost(self, link_uri, msg):
		"""Callback when disconnected after a connection has been made (i.e Crazyflie moves out of range)"""
		print('Connection to %s lost: %s' % (link_uri, msg))

	def _disconnected(self, link_uri):
		"""Callback when the Crazyflie is disconnected (called in all cases)"""
		print('Disconnected from %s' % link_uri)

	def _update_motors(self):
		self._cf.commander.send_omni(self.CompQuat, self.wx_r, self.wy_r, self.wz_r ,self.thrust)
		# print('self.CompQuat = %s '%self.CompQuat)
		# print('qw = %s  qx = %s  qy = %s  qz = %s '%(self.data_a, self.data_b,self.data_c,self.data_d))

	def _stop_crazyflie(self):
		self._cf.commander.send_stop_setpoint()
		# time.sleep(0.1)
		self._cf.close_link()

class IMUCF:
	def __init__(self, link_uri):
		""" Initialize and run the example with the specified link_uri """

		self._cf = Crazyflie(rw_cache='./cache')

		self._cf.connected.add_callback(self._connected)
		self._cf.disconnected.add_callback(self._disconnected)
		self._cf.connection_failed.add_callback(self._connection_failed)
		self._cf.connection_lost.add_callback(self._connection_lost)

		self._cf.open_link(link_uri)

		print('Connecting to %s' % link_uri)

		self.quat = np.array([1.0,0.0,0.0,0.0])
		self.quat_prev = np.array([1.0,0.0,0.0,0.0])
		self.omega = np.array([0.0,0.0,0.0])
		self.timest = 0

	def _connected(self, link_uri):

		print('Connected to %s' % link_uri)

		# The definition of the logconfig can be made before connecting
		self._lg_battery = LogConfig(name='pm', period_in_ms=1000)
		self._lg_battery.add_variable('pm.vbat','float')
		self._lg_stab = LogConfig(name='stateEstimate', period_in_ms=10)
		self._lg_stab.add_variable('stateEstimate.qw', 'float')
		self._lg_stab.add_variable('stateEstimate.qx', 'float')
		self._lg_stab.add_variable('stateEstimate.qy', 'float')
		self._lg_stab.add_variable('stateEstimate.qz', 'float')

		# Adding the configuration cannot be done until a Crazyflie is
		# connected, since we need to check that the variables we
		# would like to log are in the TOC.
		try:
			self._cf.log.add_config(self._lg_battery)
			self._lg_battery.data_received_cb.add_callback(self._battery_log_data)
			self._lg_battery.start()

			self._cf.log.add_config(self._lg_stab)
			# This callback will receive the data
			self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
			# This callback will be called on errors
			self._lg_stab.error_cb.add_callback(self._stab_log_error)
			# Start the logging
			self._lg_stab.start()
		except KeyError as e:
			print('Could not start log configuration,'
				  '{} not found in TOC'.format(str(e)))
		except AttributeError:
			print('Could not add Stabilizer log config, bad configuration.')

	def _battery_log_data(self, timestamp, data, logconf):
		battery_data = round(data['pm.vbat'], 1)
		print('Battery voltage is: |3.1V(E)| --- %s V --- |4.2V(F)|' % battery_data)
		self._lg_battery.data_received_cb.remove_callback(self._battery_log_data)

	def _stab_log_error(self, logconf, msg):
		"""Callback from the log API when an error occurs"""
		print('Error when logging %s: %s' % (logconf.name, msg))

	def _stab_log_data(self, timestamp, data, logconf):
		"""Callback froma the log API when data arrives"""
		self.quat[0] = data["stateEstimate.qw"]
		self.quat[1] = data["stateEstimate.qx"]
		self.quat[2] = data["stateEstimate.qy"]
		self.quat[3] = data["stateEstimate.qz"]

		# calculate omega by quaternions
		dq = (self.quat - self.quat_prev) / (timestamp - self.timest) * 1000
		w,x,y,z = self.quat
		om = 2 * np.dot(np.array([[w, x, y, z],
							[-x, w, z, -y],
							[-y, -z, w, x],
							[-z, y, -x, w]]), dq)
		self.omega = om[1:4]
		self.quat_prev = self.quat[:]
		self.timest = timestamp

	def _connection_failed(self, link_uri, msg):
		"""Callback when connection initial connection fails (i.e no Crazyflie
		at the specified address)"""
		print('Connection to %s failed: %s' % (link_uri, msg))

	def _connection_lost(self, link_uri, msg):
		"""Callback when disconnected after a connection has been made (i.e
		Crazyflie moves out of range)"""
		print('Connection to %s lost: %s' % (link_uri, msg))

	def _disconnected(self, link_uri):
		"""Callback when the Crazyflie is disconnected (called in all cases)"""
		print('Disconnected from %s' % link_uri)

	def _stop_crazyflie(self):
		# time.sleep(0.1)
		self._cf.close_link()

class omni_logger:
	"""
	logger class. record data and export to a csv file
	"""
	def __init__(self, folder_name='log_files'):
		self.folder_name = folder_name
		self.log_memory = [["timestamp", "qw_r", "qx_r", "qy_r", "qz_r","ch1", "ch2", "ch3", "ch4"]]

	def log_append(self, timestamp, qwr, qxr, qyr, qzr, a, b, c, d):

		self.log_memory.append([timestamp, qwr, qxr, qyr, qzr, a, b, c, d])

	# def __init__(self, folder_name='log_files'):
	# 	self.folder_name = folder_name
	# 	self.log_memory = [["timestamp", "qw_r", "qx_r", "qy_r", "qz_r", "qw", "qx", "qy", "qz", "eRx", "eRy", "eRz", "eWx", "eWy", "eWz", "Tau_x", "Tau_y", "Tau_z", 
	# 	"rollPart", "pitchPart", "yawPart", "thrustPart", "t_m1", "t_m2", "t_m3", "t_m4"]]

	# def log_append(self, timestamp, qwr, qxr, qyr, qzr, qw, qx, qy, qz, eRx, eRy, eRz, eWx, eWy, eWz, Tau_x, Tau_y, Tau_z, 
	# 	rollPart, pitchPart, yawPart, thrustPart, t_m1, t_m2, t_m3, t_m4):

	# 	self.log_memory.append([timestamp, qwr, qxr, qyr, qzr, qw, qx, qy, qz, eRx, eRy, eRz, eWx, eWy, eWz, Tau_x, Tau_y, Tau_z, 
	# 	rollPart, pitchPart, yawPart, thrustPart, t_m1, t_m2, t_m3, t_m4])

	def savelog(self):
		try:
			# Create target Directory
			os.mkdir(self.folder_name)
			print("Directory \"" + self.folder_name +  "\" Created ") 
		except:
			# print("Directory " + self.folder_name +  " already exists")
			pass
			
		with open(self.folder_name + '/log_' + time.strftime("%m%d_%H%M%S") + '.txt', 'w') as csvfile:
			writer = csv.writer(csvfile)
			writer.writerows(self.log_memory)
			print("CSV file: " + "log_" + time.strftime("%m%d_%H%M%S") + ".txt" + " created")

	def _rpy_YZX(self, quat0, quat1, quat2, quat3):
		rpy = np.array([0.0,0.0,0.0])
		pitch = np.arctan2(-2*(quat2*quat3-quat0*quat1), 1-2*(quat1**2+quat3**2))
		roll = np.arctan2(-2*(quat1*quat3-quat0*quat2), 1-2*(quat2**2+quat3**2))
		yaw = np.arcsin(2*(quat1*quat2+quat0*quat3))
		rpy = [roll,pitch,yaw]
		return rpy
	
	def _rpy_XYZ(self, quat0, quat1, quat2, quat3):
		rpy = np.array([0.0,0.0,0.0])
		roll = np.arctan2(-2*(quat2*quat3-quat0*quat1), 1-2*(quat1**2+quat3**2))
		pitch = np.arctan2(-2*(quat1*quat3-quat0*quat2), 1-2*(quat2**2+quat3**2))
		yaw = np.arcsin(2*(quat1*quat2+quat0*quat3))
		rpy = [roll,pitch,yaw]
		return rpy

	def plot_pos(self, timestamp, ch1, ch2, ch3, ch4, rpyr):

		rpy = self._rpy_XYZ(ch1, ch2, ch3, ch4)

		plt.subplot(311)
		plt.plot(timestamp, rpyr[0], 'b--', timestamp, -rpy[1], 'r')
		# plt.plot(timestamp, rpyr[0], 'b--', timestamp, qw, 'k')
		plt.ylabel('angle (rad)')
		plt.title('Roll')
		plt.grid(True)

		plt.subplot(312)
		plt.plot(timestamp, rpyr[1], 'b--', timestamp, rpy[0], 'r')
		# plt.plot(timestamp, rpyr[1], 'b--', timestamp, qx, 'k')
		plt.ylabel('angle (rad)')
		plt.title('Pitch')
		plt.grid(True)

		plt.subplot(313)
		plt.plot(timestamp, rpyr[2], 'b--', timestamp, rpy[2], 'r')
		# plt.plot(timestamp, rpyr[2], 'b--', timestamp, qy, 'k')
		plt.ylabel('angle (rad)')
		plt.title('Yaw')
		plt.grid(True)

		plt.show()

	def plot_rate(self, timestamp, ch1, ch2, ch3, ch4):

		plt.subplot(211)
		plt.plot(timestamp, ch1, 'b--', timestamp, ch2, 'r')
		plt.ylabel('wx (rad/s)')
		plt.title('Roll Rate')
		plt.grid(True)

		plt.subplot(212)
		plt.plot(timestamp, ch3, 'b--', timestamp, ch4, 'r')
		plt.ylabel('wy (rad)')
		plt.title('Pitch Rate')
		plt.grid(True)
		plt.show()
	
	def plot_channel(self, timestamp, ch1, ch2, ch3, ch4, rpyr):

		plt.subplot(221)
		plt.plot(timestamp, rpyr[0], 'b--', timestamp, ch1, 'k')
		plt.title('Channel 1')
		plt.grid(True)

		plt.subplot(222)
		plt.plot(timestamp, rpyr[1], 'b--', timestamp, ch2, 'k')
		plt.title('Channel 2')
		plt.grid(True)

		plt.subplot(223)
		plt.plot(timestamp, rpyr[2], 'b--', timestamp, ch3, 'k')
		plt.title('Channel 3')
		plt.grid(True)

		plt.subplot(224)
		plt.plot(timestamp, rpyr[2], 'b--', timestamp, ch4, 'k')
		plt.title('Channel 4')
		plt.grid(True)

		plt.show()

	def plot(self, plotTarget):
		n = len(self.log_memory)
		log_memory_array = np.asarray(self.log_memory[1:n])
		timestamp = (log_memory_array[:, 0] - log_memory_array[0, 0]) * 0.001
		qwr = log_memory_array[:, 1]
		qxr = log_memory_array[:, 2]
		qyr = log_memory_array[:, 3]
		qzr = log_memory_array[:, 4]
		ch1 = log_memory_array[:, 5]
		ch2 = log_memory_array[:, 6]
		ch3 = log_memory_array[:, 7]
		ch4 = log_memory_array[:, 8]

		rpyr = self._rpy_XYZ(qwr, qxr, qyr, qzr)
		
		if plotTarget == PlotTarget.POS:
			self.plot_pos(timestamp, ch1, ch2, ch3, ch4, rpyr)
		elif plotTarget == PlotTarget.RATE:
			# wx_r, wx, wy_r, wy
			self.plot_rate(timestamp, ch1, ch2, ch3, ch4)
		elif plotTarget == PlotTarget.CH:
			self.plot_channel(timestamp, ch1, ch2, ch3, ch4, rpyr)

class Controller:
	def __init__(self):
		self.alpha = 0
		self.beta = 0
		self.thrust = 0 # 42598
		self.thrust_constant = 0.24
		self.controller_rate = 0.01

		self.R_Bi_i_d = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
		self.R_B_Bi = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
		self.R_W_B = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
		# self.R_i_qi = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
		self.quat_r = np.array([1.0,0.0,0.0,0.0])
		self.omega_fromQuat = np.array([0.0,0.0,0.0])
		self.last_quat_r = np.array([1.0,0.0,0.0,0.0])
		self.CompQuat = 0
		self.tempAlpha = 0

		self.controller_start_time = time.time()
		self.stop_controller = False

		print("controller_start_time = %s" % self.controller_start_time)

	def AlphaBetaToQuat(self):
		# Alpha - beta meets the CrazyFlie QC body coordinate definition (Power distribution has to be modified correspondingly)
		# Rx(beta) * Ry(-alpha)
		# self.R_Bi_i_d = np.array([[np.cos(self.alpha), 0, -np.sin(self.alpha)],
		# 				[-np.sin(self.alpha)*np.sin(self.beta), np.cos(self.beta), -np.sin(self.beta)*np.cos(self.alpha)], 
		# 				[np.sin(self.alpha)*np.cos(self.beta), np.sin(self.beta), np.cos(self.alpha)*np.cos(self.beta)]])

		# Ry(-alpha) * Rx(beta)
		# self.R_Bi_i_d = np.array([[np.cos(self.alpha), -np.sin(self.alpha)*np.sin(self.beta), -np.sin(self.alpha)*np.cos(self.beta)],
		# 				[0, np.cos(self.beta), -np.sin(self.beta)], 
		# 				[np.sin(self.alpha), np.cos(self.alpha)*np.sin(self.beta), np.cos(self.alpha)*np.cos(self.beta)]])
		
		# self.R_Bi_i_d = np.array([[0, -np.cos(self.beta), np.sin(self.beta)],
		# 		[np.cos(self.alpha), -np.sin(self.alpha)*np.sin(self.beta), -np.sin(self.alpha)*np.cos(self.beta)], 
		# 		[np.sin(self.alpha), np.cos(self.alpha)*np.sin(self.beta), np.cos(self.alpha)*np.cos(self.beta)]])

		# Ry(beta) * Rx(alpha)
		self.R_Bi_i_d = np.array([[np.cos(self.beta), 0, np.sin(self.beta)],
								[np.sin(self.alpha)*np.sin(self.beta), np.cos(self.alpha), -np.sin(self.alpha)*np.cos(self.beta)], 
								[-np.cos(self.alpha)*np.sin(self.beta), np.sin(self.alpha), np.cos(self.alpha)*np.cos(self.beta)]])

		self.R_B_Bi = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
		self.R_W_B = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]]) # should be from IMUCF
		temp = np.matmul(self.R_B_Bi, self.R_Bi_i_d)
		self.R_W_i_d = np.matmul(self.R_W_B, temp)
		self.quat_r = utils2.rot2quat(self.R_W_i_d)
		self.omega_fromQuat = utils2.omega(self.quat_r,self.last_quat_r,self.controller_rate)
		self.last_quat_r = self.quat_r
		self.CompQuat = utils2.quatCompress(self.quat_r)

	def run(self):
		last_loop_time = time.time()
		while not self.stop_controller:
			current_time = time.time()
			if current_time - last_loop_time > self.controller_rate:
				if current_time - self.controller_start_time < 1:
					self.alpha = 0
					self.beta = 0
					self.thrust = 0
				elif current_time - self.controller_start_time < 3:
					self.alpha = 0.3
					self.beta = 0.0
					# self.alpha = 0.1*(current_time - self.controller_start_time)
					# self.beta = 0.2*(current_time - self.controller_start_time)
					self.thrust = self.thrust_constant
				elif current_time - self.controller_start_time < 4:
					self.alpha = 0.0
					self.beta = 0.0
					self.thrust = self.thrust_constant
				elif current_time - self.controller_start_time < 6:
					self.alpha = 0.0
					self.beta = 0.3
					# self.beta = 0.1*(current_time - self.controller_start_time)
					self.thrust = self.thrust_constant
				elif current_time - self.controller_start_time < 8:
					self.alpha = 0.0
					self.beta = 0.0
					self.thrust = self.thrust_constant
				else:
					self.alpha = 0; self.beta = 0; self.thrust = 0
				self.AlphaBetaToQuat()
				last_loop_time = current_time
			else:
				time.sleep(0.002)

if __name__ == '__main__':

	# Initialize the low-level drivers (don't list the debug drivers)
	cflib.crtp.init_drivers(enable_debug_driver=False)
	# Scan for Crazyflies and use the first one found
	print('Scanning interfaces for Crazyflies...')
	available = cflib.crtp.scan_interfaces()

	if True:
		Omni = OmniAttCF('radio://0/80/2M/E7E7E7E7E6', 0)
		time.sleep(0.25)
		# imu = IMUCF('radio://0/80/2M/E7E7E7E7EB')
		logger = omni_logger(folder_name='log_test_single_1215')
		time.sleep(4)

		cmd_rate = 0.005
		t = 0

		ctrl = Controller()
		ctrl_thread = Thread(target=ctrl.run)
		ctrl_thread.start()
		start_time = time.time()

		print("start_time = %s" % start_time)

		while t < 8:
			Omni.CompQuat = ctrl.CompQuat
			# Omni.wx_r = ctrl.omega_fromQuat[0]
			Omni.wx_r = ctrl.beta
			Omni.wy_r = ctrl.omega_fromQuat[1]
			Omni.wz_r = ctrl.omega_fromQuat[2]
			Omni.thrust = ctrl.thrust
			# le.base_q = imu.quat
			# En ég stend alltaf upp
			Omni._update_motors()
			t += cmd_rate
			time.sleep(cmd_rate)

			if Omni.plotTarget == PlotTarget.POS:
				# plot pos
				logger.log_append(Omni.timest, ctrl.quat_r[0], ctrl.quat_r[1], ctrl.quat_r[2], ctrl.quat_r[3], Omni.qw_IMU, Omni.qx_IMU, Omni.qy_IMU, Omni.qz_IMU)
			elif Omni.plotTarget == PlotTarget.RATE:
				# plot rate
				logger.log_append(Omni.timest, ctrl.quat_r[0], ctrl.quat_r[1], ctrl.quat_r[2], ctrl.quat_r[3], Omni.wx_ref, Omni.wx, Omni.wy_ref, Omni.wy)
			# plot channel
			# logger.log_append(Omni.timest, ctrl.quat_r[0], ctrl.quat_r[1], ctrl.quat_r[2], ctrl.quat_r[3], 
							# Omni.qw_IMU, Omni.qx_IMU, Omni.qy_IMU, Omni.qz_IMU)
							# Omni.qw_IMU, Omni.KRx, Omni.eRx, Omni.wx_ref)
							# Omni.wx_ref, Omni.wy_ref, Omni.wz_ref, Omni.eRx )# Omni.eRz
							# Omni.eWx, Omni.eWy, Omni.eWz, Omni.Tau_x)
							# Omni.Tau_x, Omni.Tau_y, Omni.Tau_z, Omni.eWx)
							# Omni.rollPArt, Omni.pitchPart, Omni.yawPart, Omni.thrustPart)
							# Omni.t_m1, Omni.t_m2, Omni.t_m3, Omni.t_m4)
			

		Omni._stop_crazyflie()
		# imu._stop_crazyflie()
		ctrl.stop_controller = True
		logger.savelog()
		logger.plot(Omni.plotTarget)


	else:
		print('No Crazyflies found, cannot run')
