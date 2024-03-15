import logging
import time
import os
import sys
import csv
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
		self.log_memory = [["timestamp", "e", "u"]]

	def log_append(self, timestamp, e, u):
		"""
		timestamp = round(timestamp,3)
		e = round(e,3)
		u = round(u,3)
		"""
		self.log_memory.append([timestamp, e, u])

	def stop(self):
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
			print("CSV file" + "log_" + time.strftime("%m%d_%H%M%S") + ".txt" + " created")


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

		self.alpha = 0
		self.beta = 0
		self.thrust = 0

		self.error_beta = 0
		self.u_beta = 0
		self.timest = 0

	def _connected(self, link_uri):

		print('Connected to %s' % link_uri)

		# The definition of the logconfig can be made before connecting
		
		self._lg_stab = LogConfig(name='sctrl', period_in_ms=10)
		# self._lg_stab.add_variable('sctrl.t_be')
		# self._lg_stab.add_variable('sctrl.t_ae')
		# self._lg_stab.add_variable('sctrl.t_bin')

		
		# self._lg_stab.add_variable('sctrl.t_m1')
		# self._lg_stab.add_variable('sctrl.t_m2')
		# self._lg_stab.add_variable('sctrl.t_m3')
		# self._lg_stab.add_variable('sctrl.t_m4')
		

		self._lg_stab.add_variable('sctrl.error_beta', 'float')
		self._lg_stab.add_variable('sctrl.u_beta', 'float')
		# self._lg_stab.add_variable('sctrl.error_alpha')
		# self._lg_stab.add_variable('sctrl.u_alpha')
		# self._lg_stab.add_variable('sctrl.t_pbout')
		# self._lg_stab.add_variable('sctrl.t_ibout')
		# self._lg_stab.add_variable('sctrl.t_dbout')
		
		"""
		self._lg_stab = LogConfig(name='motor', period_in_ms=40)
		self._lg_stab.add_variable('motor.m1')
		self._lg_stab.add_variable('motor.m2')
		self._lg_stab.add_variable('motor.m3')
		self._lg_stab.add_variable('motor.m4')
		"""


		# Adding the configuration cannot be done until a Crazyflie is
		# connected, since we need to check that the variables we
		# would like to log are in the TOC.
		try:
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

	def _stab_log_error(self, logconf, msg):
		"""Callback from the log API when an error occurs"""
		print('Error when logging %s: %s' % (logconf.name, msg))

	def _stab_log_data(self, timestamp, data, logconf):
		"""Callback froma the log API when data arrives"""
		self.timest = timestamp
		self.error_beta = data["sctrl.error_beta"]
		self.u_beta = data["sctrl.u_beta"]

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
		self._cf.commander.send_twod(0, 1, 0, 0, 0, self.alpha, self.beta, self.thrust)

	def _stop_crazyflie(self):
		self._cf.commander.send_stop_setpoint()
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

	if len(available) > 0:
		le = ThrustRamp(available[0][0])
		logger = ab_logger(folder_name='log_test_0729')
		cmd_rate = 0.02
		alpha = 0
		beta = 0
		thrust = 0
		t = 0
		time.sleep(2)

		while t < 1:
			le._update_motors()
			t += cmd_rate
			time.sleep(cmd_rate)
			logger.log_append(le.timest, le.error_beta, le.u_beta)

		while t < 3:
			thrust = 10000
			beta = 0.5
			le.beta = beta
			le.thrust = thrust
			le._update_motors()
			t += cmd_rate
			time.sleep(cmd_rate)
			logger.log_append(le.timest, le.error_beta, le.u_beta)

		le._stop_crazyflie()
		logger.stop()


	else:
		print('No Crazyflies found, cannot run')
