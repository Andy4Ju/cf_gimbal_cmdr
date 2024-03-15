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

logging.basicConfig(level=logging.ERROR)

class ab_logger:
	"""
	logger class. record data and export to a csv file
	"""
	def __init__(self, folder_name='log_files'):
		self.folder_name = folder_name
		self.log_memory = [["timestamp", "angle", "torque", "thrust", "t_m1"]]

	def log_append(self, timestamp, da, db, dc, dd):
		self.log_memory.append([timestamp, da, db, dc, dd])

	def savelog(self):
		# print(self.log_memory)
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

	def plot(self):
		n = len(self.log_memory)
		log_memory_array = np.asarray(self.log_memory[1:n])
		timestamp = (log_memory_array[:, 0] - log_memory_array[0, 0]) * 0.001
		da = log_memory_array[:, 1]
		db = log_memory_array[:, 2]
		dc = log_memory_array[:, 3]
		dd = log_memory_array[:, 4]

		plt.subplot(121)
		plt.plot(timestamp, da, 'k')
		plt.title('angle (rad)')
		plt.grid(True)

		'''
		plt.subplot(122)
		plt.plot(timestamp, db, 'k')
		plt.title('torque (N/m)')
		plt.grid(True)
		'''

		plt.subplot(122)
		plt.plot(timestamp, db, 'k')
		plt.title('torque (N/m)')
		plt.grid(True)


		plt.show()


class ThrustRamp:
	def __init__(self, link_uri):
		""" Initialize and run the example with the specified link_uri """

		self._cf = Crazyflie(rw_cache='./cache')

		self._cf.connected.add_callback(self._connected)
		self._cf.disconnected.add_callback(self._disconnected)
		self._cf.connection_failed.add_callback(self._connection_failed)
		self._cf.connection_lost.add_callback(self._connection_lost)

		self._cf.open_link(link_uri)

		print('Connecting to %s' % link_uri)

		self.alpha = 0 # desired tau_x
		self.thrust = 0

		self.t_alphae = 0
		self.t_betae = 0
		self.timest = 0
		self.da = 0

	def _connected(self, link_uri):

		print('Connected to %s' % link_uri)

		# The definition of the logconfig can be made before connecting
		self._lg_battery = LogConfig(name='pm', period_in_ms=1000)
		self._lg_battery.add_variable('pm.vbat','float')
		self._lg_stab = LogConfig(name='sctrl', period_in_ms=10)
		self._lg_stab.add_variable('sctrl.t_ae', 'float')
		self._lg_stab.add_variable('sctrl.t_m1', 'float')
		# self._lg_stab.add_variable('sctrl.t_be', 'float')

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
		self.timest = timestamp
		self.t_alphae = data["sctrl.t_ae"]
		self.da = data["sctrl.t_m1"]
		# self.t_betae = data["sctrl.t_be"]

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

	def _update_motors(self):
		self._cf.commander.send_twod(0, 1, 0, 0, 0, self.alpha, 0, self.thrust)

	def _stop_crazyflie(self):
		self._cf.commander.send_twod(0, 1, 0, 0, 0, 0, 0, 0)
		# time.sleep(0.1)
		self._cf.close_link()



if __name__ == '__main__':

	# Initialize the low-level drivers (don't list the debug drivers)
	cflib.crtp.init_drivers(enable_debug_driver=False)
	# Scan for Crazyflies and use the first one found
	print('Scanning interfaces for Crazyflies...')
	available = cflib.crtp.scan_interfaces()
	print('Crazyflies found:')
	for i in available:
		print(i[0])

	if True: # len(available) > 0:
		le = ThrustRamp('radio://0/80/2M/E7E7E7E7E8')
		# le = ThrustRamp(available[0][0])
		logger = ab_logger(folder_name='sysid_1dof_0910_3')
		time.sleep(4)

		nom_thrust_kg = 0 # 0 ~ 0.06 
		nom_tau_kg = 0 # 0 ~ +- 0.06

		nom_thrust = nom_thrust_kg * 9.81 # 0 ~ 0.5886
		nom_tau = nom_tau_kg * 9.81 * 0.03165 # 0 ~ +- 0.01862919

		print('thrust = %s tau = %s' % (nom_thrust, nom_tau))

		log_rate = 0.005
		cmd_rate = 0.05
		t = 0
		count = 0
		rate_ratio = cmd_rate/log_rate
		start_time = time.time()
		print('start at %s' % start_time)

		while t < 6:
			le.thrust = nom_thrust
			le.alpha = 0
			if count % rate_ratio == 0:
				le._update_motors()
				count = 0
			count += 1
			t += log_rate
			time.sleep(log_rate)
			logger.log_append(le.timest, le.t_alphae, le.alpha, le.thrust, le.da)

		while t < 0:
			le.thrust = nom_thrust
			le.alpha = nom_tau
			if count % rate_ratio == 0:
				le._update_motors()
				count = 0
			count += 1
			t += log_rate
			time.sleep(log_rate)
			logger.log_append(le.timest, le.t_alphae, le.alpha, le.thrust, le.da)

		le._stop_crazyflie()
		logger.savelog()
		logger.plot()


	else:
		print('No Crazyflies found, cannot run')
