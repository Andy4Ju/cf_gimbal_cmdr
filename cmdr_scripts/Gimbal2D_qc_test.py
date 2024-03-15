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

logging.basicConfig(level=logging.ERROR)

CMD_RATE = 0.005

class ab_logger:
	"""
	logger class. record data and export to a csv file
	"""
	def __init__(self, folder_name='log_files'):
		self.folder_name = folder_name
		self.log_memory = [["timestamp", "alpha_d", "beta_d", "data_a", "data_b", "data_c", "data_d", "roll", "pitch", "yaw"]]

	def log_append(self, timestamp, ad, bd, da, db, dc, dd, r, p, y):
		"""
		timestamp = round(timestamp,3)
		e = round(e,3)
		d = round(d,3)
		"""
		self.log_memory.append([timestamp, ad, bd, da, db, dc, dd, r, p, y])

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

	def _getStepInfo(self, ref, fbk, Ts):
		l = len(ref)
		Step_Value = 0
		for x in range(2, l):
			if ref[x] != 0 and ref[x-1] == 0:
				t0_tick = x
				Step_Value = ref[x]
			if ref[x] == 0 and ref[x-1] != 0:
				tf_tick = x

		if Step_Value != 0:
			Tr0_tick = 0
			Trf_tick = 0
			Ymax = 0
			for y in range(t0_tick, tf_tick):
				if abs(fbk[y]) > abs(0.1 * Step_Value) and abs(fbk[y-1]) <= abs(0.1 * Step_Value) and Tr0_tick == 0:
					Tr0_tick = y
				if abs(fbk[y]) > abs(0.9 * Step_Value) and abs(fbk[y-1]) <= abs(0.9 * Step_Value) and Trf_tick == 0:
					Trf_tick = y
				if abs(fbk[y]) > Ymax:
					Ymax = fbk[y]
			if Tr0_tick != 0 and Trf_tick != 0:
				Tr = (Trf_tick - Tr0_tick) * Ts
			else:
				Tr = -1
			Mp = (Ymax / Step_Value - 1) * 100
		else:
			Tr = 0
			Mp = 0
   
		# print('Tr0_tick = %s, Trf_tick = %s'%(Tr0_tick, Trf_tick))
		# print('Ymax = ',Ymax)
		return [Tr, Mp]

	def plot(self):
		n = len(self.log_memory)
		# print('log_memory len = ', n)
		log_memory_array = np.asarray(self.log_memory[1:n])
		timestamp = (log_memory_array[:, 0] - log_memory_array[0, 0]) * 0.001
		m = len(timestamp)
		# print('timestamp len = ', m)
		ad = log_memory_array[:, 1]
		bd = log_memory_array[:, 2]
		da = log_memory_array[:, 3]
		db = log_memory_array[:, 4]
		dc = log_memory_array[:, 5]
		dd = log_memory_array[:, 6]
		r = log_memory_array[:, 7]
		p = log_memory_array[:, 8]
		y = log_memory_array[:, 9]

		[Alpha_Tr, Alpha_Mp] = self._getStepInfo(ad, da, CMD_RATE)
		[Beta_Tr, Beta_Mp] = self._getStepInfo(bd, dc, CMD_RATE)

		print('Alpha_Tr = %s seconds, Alpha_Mp = %s percent'%(Alpha_Tr, Alpha_Mp))
		print('Beta_Tr = %s seconds, Beta_Mp = %s percent'%(Beta_Tr, Beta_Mp))

		plt.subplot(211)
		plt.plot(timestamp, ad, 'b--', timestamp, da, 'k')
		plt.ylabel('angle (rad)')
		plt.title('alpha')
		plt.legend(['alpha_ref','alpha'])
		plt.grid(True)

		plt.subplot(212)
		plt.plot(timestamp, bd, 'b--', timestamp, dc, 'k')
		plt.ylabel('angle (rad)')
		plt.title('beta')
		plt.legend(['beta_ref','beta'])
		plt.grid(True)

		# alpha_error = ad - da
		# beta_error = bd - dc
		# plt.subplot(211)
		# plt.plot(timestamp, ad, 'b--', timestamp, da, 'k', timestamp, alpha_error, 'r')
		# plt.ylabel('angle (rad)')
		# plt.title('alpha')
		# plt.legend(['alpha_ref','alpha','error'])
		# plt.grid(True)

		# plt.subplot(212)
		# plt.plot(timestamp, bd, 'b--', timestamp, dc, 'k', timestamp, beta_error, 'r')
		# plt.ylabel('angle (rad)')
		# plt.title('beta')
		# plt.legend(['beta_ref','beta','error'])
		# plt.grid(True)

		plt.show()
class CrazyflieGimbal2D:
	def __init__(self, link_uri, index):
		""" Initialize and run the example with the specified link_uri """

		self._cf = Crazyflie(rw_cache='./cache')

		self._cf.fully_connected.add_callback(self._connected)
		self._cf.disconnected.add_callback(self._disconnected)
		self._cf.connection_failed.add_callback(self._connection_failed)
		self._cf.connection_lost.add_callback(self._connection_lost)

		self._cf.open_link(link_uri)

		print('Connecting to %s' % link_uri)

		self.alpha = 0
		self.beta = 0
		self.thrust = 0
		self.index = index
		self.base_q = np.array([1,0,0,0])

		self.data_a = 0
		self.data_b = 0
		self.data_c = 0
		self.data_d = 0
		self.timest = 0

		self.gain_name = ['pgaina', 'igaina', 'dgaina', 'pgainb', 'igainb', 'dgainb', 
							'pgainas', 'igainas', 'dgainas', 'pgainbs', 'igainbs', 'dgainbs']
  
		self.gain_value = [15.5, 0.26, 0, 17.2, 0.157, 0, 240, 240, 5.5, 280, 280, 6.3]

	def _connected(self, link_uri):

		print('Connected to %s' % link_uri)

		# modify PID gains parameters
		self._cf.param.add_update_callback(group='sparam_Gimbal2D', name=None,
											cb=self._param_callback)
		for n in self.gain_name:
			ind = self.gain_name.index(n)
			self._cf.param.set_value('sparam_Gimbal2D.{}'.format(n), '{}'.format(self.gain_value[ind]))
			self._cf.param.remove_update_callback(group='controller_tune', name=n,
												cb=self._param_callback)

		# The definition of the logconfig can be made before connecting
		self._lg_battery = LogConfig(name='pm', period_in_ms=1000)
		self._lg_battery.add_variable('pm.vbat','float')
		self._lg_stab = LogConfig(name='sctrl_Gimbal2D', period_in_ms=10)

		self._lg_stab.add_variable('sctrl_Gimbal2D.alpha', 'float')
		self._lg_stab.add_variable('sctrl_Gimbal2D.alphas', 'float')
		self._lg_stab.add_variable('sctrl_Gimbal2D.beta', 'float')
		self._lg_stab.add_variable('sctrl_Gimbal2D.betas', 'float')
		# self._lg_stab.add_variable('sctrl_Gimbal2D.t_m1', 'float')
		# self._lg_stab.add_variable('sctrl_Gimbal2D.t_m2', 'float')
		# self._lg_stab.add_variable('sctrl_Gimbal2D.t_m3', 'float')
		# self._lg_stab.add_variable('sctrl_Gimbal2D.t_m4', 'float')

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
		self.data_a = data["sctrl_Gimbal2D.alpha"]
		self.data_b = data["sctrl_Gimbal2D.alphas"]
		self.data_c = data["sctrl_Gimbal2D.beta"]
		self.data_d = data["sctrl_Gimbal2D.betas"]
		# self.data_a = data["sctrl_Gimbal2D.t_m1"]
		# self.data_b = data["sctrl_Gimbal2D.t_m2"]
		# self.data_c = data["sctrl_Gimbal2D.t_m3"]
		# self.data_d = data["sctrl_Gimbal2D.t_m4"]

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
		self._cf.commander.send_twod(self.index, self.base_q[0], self.base_q[1], self.base_q[2], self.base_q[3], self.alpha, self.beta, self.thrust)

	def _stop_crazyflie(self):
		self._cf.commander.send_stop_setpoint()
		self._cf.close_link()

class StepReferenceGenerator:
	def __init__(self):
		self.alpha = 0
		self.beta = 0
		self.thrust = 0 # 42598
		self.thrust_constant = 0.44
		self.controller_rate = 0.01

		self.controller_start_time = time.time()
		self.stop_controller = False

		print("controller_start_time = %s" % self.controller_start_time)

	def run(self):
		last_loop_time = time.time()
		while not self.stop_controller:
			current_time = time.time()
			if current_time - last_loop_time > self.controller_rate:
				if current_time - self.controller_start_time < 0:
					self.alpha = 0
					self.beta = 0
					self.thrust = 0
				elif current_time - self.controller_start_time < 1:
					self.alpha = 0
					self.beta = 0
					self.thrust = self.thrust_constant
				elif current_time - self.controller_start_time < 2:
					self.alpha = 0.4
					self.beta = 0
					self.thrust = self.thrust_constant	
				elif current_time - self.controller_start_time < 4:
					self.alpha = 0
					self.beta = 0
					self.thrust = self.thrust_constant
				elif current_time - self.controller_start_time < 5:
					self.alpha = 0
					self.beta = 0.4
					self.thrust = self.thrust_constant
				elif current_time - self.controller_start_time < 6:
					self.alpha = 0.0
					self.beta = 0.0
					self.thrust = self.thrust_constant
				else:
					self.alpha = 0; self.beta = 0; self.thrust = 0
				last_loop_time = current_time
			else:
				time.sleep(0.002)
class TrajReferenceGenerator:
	def __init__(self):
		self.alpha = 0
		self.beta = 0
		self.thrust = 0 # 42598
		self.thrust_constant = 0.16
		self.controller_rate = 0.01

		self.controller_start_time = time.time()
		self.stop_controller = False

		print("controller_start_time = %s" % self.controller_start_time)

	def run(self):
		last_loop_time = time.time()
		while not self.stop_controller:
			current_time = time.time()
			if current_time - last_loop_time > self.controller_rate:
				tnow = current_time - self.controller_start_time
				self.test_thrust(tnow)
				last_loop_time = current_time
			else:
				time.sleep(0.002)
    
	def test_thrust(self, tnow):
		print('tnow = ', tnow)
		if tnow < 3:
			self.alpha = 0
			self.beta = 0
			self.thrust = self.thrust_constant*tnow/3
		elif tnow < 6:
			self.alpha = 0
			self.beta = 0
			self.thrust = self.thrust_constant
		elif tnow < 9:
			self.alpha = 0
			self.beta = 0
			self.thrust = self.thrust_constant*(1-(tnow-6)/3)
		else:
			self.alpha = 0; self.beta = 0; self.thrust = 0
	
	def test_betalpha(self, tnow):
		if tnow < 3:
			self.alpha = 0
			self.beta = 0
			self.thrust = self.thrust_constant*tnow/3
		elif tnow < 6:
			self.alpha = 0
			self.beta = np.pi/4*(tnow-3)/3
			self.thrust = self.thrust_constant
		elif tnow < 9:
			self.alpha = 0
			self.beta = np.pi/4
			self.thrust = self.thrust_constant	
		elif tnow < 12:
			self.alpha = np.pi/6*(tnow-9)/3
			self.beta = np.pi/4
			self.thrust = self.thrust_constant
		elif tnow < 15:
			self.alpha = np.pi/6
			self.beta = np.pi/4
			self.thrust = self.thrust_constant
		elif tnow < 18:
			self.alpha = np.pi/6
			self.beta = np.pi/4
			self.thrust = self.thrust_constant*(1-(tnow-15)/3)
		else:
			self.alpha = 0; self.beta = 0; self.thrust = 0



if __name__ == '__main__':

	# Initialize the low-level drivers (don't list the debug drivers)
	cflib.crtp.init_drivers(enable_debug_driver=False)
	# Scan for Crazyflies and use the first one found
	print('Scanning interfaces for Crazyflies...')
	available = cflib.crtp.scan_interfaces()

	if True:
		# le = CrazyflieGimbal2D('radio://0/80/2M/E7E7E7E7E9', 0)
		le = CrazyflieGimbal2D('radio://0/120/2M/E7E7E7E7EF', 0)
		time.sleep(0.25)
		logger = ab_logger(folder_name='log_test_single_1205')
		time.sleep(4)

		cmd_rate = CMD_RATE # 200Hz
		t = 0

		RefGen = StepReferenceGenerator()
		# RefGen = TrajReferenceGenerator()
		ctrl_thread = Thread(target=RefGen.run)
		ctrl_thread.start()
		start_time = time.time()

		print("start_time = %s" % start_time)

		while t < 6:
			le.alpha = RefGen.alpha
			le.beta = RefGen.beta
			le.thrust = RefGen.thrust
			le._update_motors()

			t += cmd_rate
			time.sleep(cmd_rate)
			logger.log_append(le.timest, RefGen.alpha, RefGen.beta, le.data_a, le.data_b, le.data_c, le.data_d, 0, 0, 0)
		le._stop_crazyflie()
		RefGen.stop_controller = True
		logger.savelog()
		logger.plot()

	else:
		print('No Crazyflies found, cannot run')
