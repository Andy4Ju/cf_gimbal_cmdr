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
		self.log_memory = [["timestamp", "alpha_d", "beta_d", "data_a", "data_b", "data_c", "data_d"]]

	def log_append(self, timestamp, ad, bd, da, db, dc, dd):
		"""
		timestamp = round(timestamp,3)
		e = round(e,3)
		d = round(d,3)
		"""
		self.log_memory.append([timestamp, ad, bd, da, db, dc, dd])

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
		ad = log_memory_array[:, 1]
		bd = log_memory_array[:, 2]
		da = log_memory_array[:, 3]
		db = log_memory_array[:, 4]
		dc = log_memory_array[:, 5]
		dd = log_memory_array[:, 6]
		plt.subplot(221)
		plt.plot(timestamp, ad, 'b--', timestamp, da, 'k')
		plt.ylabel('angle (rad)')
		plt.title('alpha')
		plt.grid(True)

		plt.subplot(222)
		plt.plot(timestamp, db, 'k')
		plt.ylabel('angle (rad)')
		plt.title('error alphas')
		plt.grid(True)

		plt.subplot(223)
		plt.plot(timestamp, dc, 'k')
		plt.ylabel('torque (Nm)')
		plt.xlabel('time (s)')
		plt.title('u alpha')
		plt.grid(True)

		plt.subplot(224)
		plt.plot(timestamp, dd, 'k')
		plt.ylabel('torque (Nm)')
		plt.xlabel('time (s)')
		plt.title('alpha gyro')
		plt.grid(True)

		plt.show()


class ThrustRamp:
	def __init__(self, link_uri, index):
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
		self.index = index

		self.data_a = 0
		self.data_b = 0
		self.data_c = 0
		self.data_d = 0
		self.timest = 0

		c1 = 4; c2 = 1.5
		self.gain_name = ['pgaina', 'igaina', 'dgaina', 'pgainb', 'igainb', 'dgainb', 
							'pgainas', 'igainas', 'dgainas', 'pgainbs', 'igainbs', 'dgainbs',
							's_tx', 's_ty', 's_tz']
		self.gain_value = [1200, 10, 1, 1400, 10, 0, 
							0.00007/c1, 0.00006/c1, 0.000003/c1, 0.00007/c1, 0.00006/c1, 0.000003/c1,
							1, 1, 1]

	def _connected(self, link_uri):

		print('Connected to %s' % link_uri)

		# modify PID gains parameters
		
		self._cf.param.add_update_callback(group='sparam_ppid', name=None,
											cb=self._param_callback)
		for n in self.gain_name:
			ind = self.gain_name.index(n)
			self._cf.param.set_value('sparam_ppid.{}'.format(n), '{}'.format(self.gain_value[ind]))
			# self._cf.param.remove_update_callback(group='controller_tune', name=n,
			# 									cb=self._param_callback)

		# The definition of the logconfig can be made before connecting
		self._lg_battery = LogConfig(name='pm', period_in_ms=1000)
		self._lg_battery.add_variable('pm.vbat','float')
		self._lg_stab = LogConfig(name='sctrl_ppid', period_in_ms=10)
		self._lg_stab.add_variable('sctrl_ppid.t_ae', 'float')
		self._lg_stab.add_variable('sctrl_ppid.e_alphas', 'float')
		self._lg_stab.add_variable('sctrl_ppid.u_alpha', 'float')
		self._lg_stab.add_variable('sctrl_ppid.a_gyro', 'float')

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
		self.data_a = data["sctrl_ppid.t_ae"]
		self.data_b = data["sctrl_ppid.e_alphas"]
		self.data_c = data["sctrl_ppid.u_alpha"]
		self.data_d = data["sctrl_ppid.a_gyro"]

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
		self._cf.commander.send_twod(self.index, 1, 0, 0, 0, self.alpha, self.beta, self.thrust)

	def _stop_crazyflie(self):
		self._cf.commander.send_stop_setpoint()
		# time.sleep(0.1)
		self._cf.close_link()

class Controller:
	def __init__(self):
		self.alpha = 0
		self.beta = 0
		self.thrust = 0 # 42598
		self.controller_rate = 0.01

		self.controller_start_time = time.time()
		self.stop_controller = False

		print("controller_start_time = %s" % self.controller_start_time)

	def run(self):
		last_loop_time = time.time()
		while not self.stop_controller:
			if time.time() - last_loop_time > self.controller_rate:
				if time.time() - self.controller_start_time > 3:
					self.thrust = 0.15

				if time.time() - self.controller_start_time > 6:
					if time.time() - self.controller_start_time > 9:
						self.beta = 0.5
						self.alpha = 0.5
					else:
						self.beta = 0.5 # 0.2 # 12 deg
						self.alpha = 0.0
				last_loop_time = time.time()
			else:
				time.sleep(0.001)




if __name__ == '__main__':

	# Initialize the low-level drivers (don't list the debug drivers)
	cflib.crtp.init_drivers(enable_debug_driver=False)
	# Scan for Crazyflies and use the first one found
	print('Scanning interfaces for Crazyflies...')
	available = cflib.crtp.scan_interfaces()
	print('Crazyflies found:')
	for i in available:
		print(i[0])

	if True:
		le = ThrustRamp('radio://0/100/2M/E7E7E7E7EA', 3)
		logger = ab_logger(folder_name='log_test_0813')
		time.sleep(4)

		cmd_rate = 0.005
		t = 0

		ctrl = Controller()
		ctrl_thread = Thread(target=ctrl.run)
		ctrl_thread.start()
		start_time = time.time()

		print("start_time = %s" % start_time)

		while t < 12:
			le.alpha = ctrl.alpha
			le.beta = ctrl.beta
			le.thrust = ctrl.thrust
			le._update_motors()
			t += cmd_rate
			time.sleep(cmd_rate)
			logger.log_append(le.timest, ctrl.alpha, ctrl.beta, le.data_a, le.data_b, le.data_c, le.data_d)

		le._stop_crazyflie()
		ctrl.stop_controller = True
		# logger.savelog()
		logger.plot()


	else:
		print('No Crazyflies found, cannot run')
