"""
Single Crazyflie test
"""
import logging
import time
import sys
import numpy as np
from threading import Thread
from cflib.crazyflie.log import LogConfig
from logger import ab_logger
from ReferenceGenerator import StepReferenceGenerator
from ReferenceGenerator import TrajReferenceGenerator

import cflib
from cflib.crazyflie import Crazyflie
from parameter import URL, CONTROLLER, REFTYPE

logging.basicConfig(level=logging.ERROR)

THRUST_CONST = 0.3 # 0 ~ 0.6
CMD_RATE = 0.005
DATA_FOLDER_NAME = 'log_'+time.strftime("%m%d")
REF_TYPE = 1
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
  
		self.gain_value = [15.5, 0.26, 0, 17.2, 0.157, 0, 200, 200, 5.5, 200, 200, 6.3]

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

if __name__ == '__main__':

	# Initialize the low-level drivers (don't list the debug drivers)
	cflib.crtp.init_drivers(enable_debug_driver=False)
	# Scan for Crazyflies and use the first one found
	print('Scanning interfaces for Crazyflies...')
	available = cflib.crtp.scan_interfaces()

	if True:
		# le = CrazyflieGimbal2D(, 0)
		le = CrazyflieGimbal2D(URL.QC_GREY_ORANGE_URL.value, 0)
		time.sleep(0.25)
		logger = ab_logger(CMD_RATE, folder_name=DATA_FOLDER_NAME)
		time.sleep(4)

		cmd_rate = CMD_RATE # 200Hz
		t = 0
		if REF_TYPE == REFTYPE.STEP.value:
			RefGen = StepReferenceGenerator(THRUST_CONST)
			T_final = 6
		elif REF_TYPE == REFTYPE.RAMP.value:
			RefGen = TrajReferenceGenerator(THRUST_CONST)
			T_final = 20

		ctrl_thread = Thread(target=RefGen.run)
		ctrl_thread.start()
		start_time = time.time()

		print("start_time = %s" % start_time)

		while t < T_final:
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
		logger.plot(REF_TYPE)
	else:
		print('No Crazyflies found, cannot run')
