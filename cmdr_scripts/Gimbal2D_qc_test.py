"""
Gimbal controller test using crazyflie
"""
import logging
import time
import numpy as np
from threading import Thread
from cflib.crazyflie.log import LogConfig
from logger import ab_logger
from ReferenceGenerator import *
from parameter import *
import math
import cflib
from cflib.crazyflie import Crazyflie

logging.basicConfig(level=logging.ERROR)

'''Usage: Follow the steps before running this script'''
'''===================================================================================================='''
'''Make sure your dependencies are ready, please refer to: https://github.com/SFWen2/cf_gimbal_cmdr/blob/main/README.md '''

'''Set the URL of your crazyflie target, add a new URL in parameter.py '''
ControlTarget = URL.QC_ITRI_URL.value

'''Assign Thrust Constant 0 ~ 0.58 N '''
THRUST_CONST = 0.2

'''Assign the reference type, 1 = step, 2 = ramp. Modify ReferenceGenerator.py if you have other references'''
RefType = REF_TYPE.REF_TYPE_PWM.value 

'''Assign the controller type, 5= singleppid, 7=gimbal2D.'''
ControllerType = CONTROLLER_TYPE.CONTROLLER_TYPE_GIMBAL2D.value # 5= singleppid, 7=gimbal2D
SubGimbal2DType = SUB_GIMBAL2D_TYPE.SUB_GIMBAL2D_TYPE_PWMTEST.value

'''Assign the date log type, angular position / velocity or pwm command'''
LogType = LOG_TYPE.LOG_TYPE_PWM_CMD.value

'''Assign the folder name of logged data, the default name is log_date'''
DATA_FOLDER_NAME = 'log_' + time.strftime("%m%d")

'''Assign the command rate'''
CMD_RATE = 0.005

'''Run the script'''
'''===================================================================================================='''

class CrazyflieGimbal2D:
	def __init__(self, link_uri, index, controller_type, log_type):
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
		self.group_name = ''
		self.set_group = ''
		self.config_name = ''
		self.data_a_name = ''
		self.data_b_name = ''
		self.data_c_name = ''
		self.data_d_name = ''
		self.timest = 0
		self.data_a = 0
		self.data_b = 0
		self.data_c = 0
		self.data_d = 0

		# init parameter and scopes
		if controller_type == CONTROLLER_TYPE.CONTROLLER_TYPE_GIMBAL2D.value:
			self.group_name = 'sparam_Gimbal2D'
			self.config_name ='sctrl_Gimbal2D'
			self.set_group = 'sparam_Gimbal2D.{}'

			if SubGimbal2DType == SUB_GIMBAL2D_TYPE.SUB_GIMBAL2D_TYPE_PID.value or SubGimbal2DType == SUB_GIMBAL2D_TYPE.SUB_GIMBAL2D_TYPE_PID_JALPHA.value:
				self.gain_name = ['pgaina', 'igaina', 'dgaina', 'pgainb', 'igainb', 'dgainb', 
									'pgainas', 'igainas', 'dgainas', 'pgainbs', 'igainbs', 'dgainbs', 'cmode']
				self.gain_value = [15.5, 0.26, 0, 17.2, 0.157, 0, 200, 200, 5.5, 240, 240, 7.3, SubGimbal2DType]

			elif SubGimbal2DType == SUB_GIMBAL2D_TYPE.SUB_GIMBAL2D_TYPE_OFL.value:
				self.gain_name = ['ofl_ld1','ofl_ld2','cmode']
				self.gain_value = [-60, -10, SubGimbal2DType]

			elif SubGimbal2DType == SUB_GIMBAL2D_TYPE.SUB_GIMBAL2D_TYPE_NSF.value:
				self.gain_name = ['nsf_K11','nsf_K12','nsf_K13','nsf_K14','nsf_K21','nsf_K22','nsf_K23','nsf_K24','cmode']
				self.gain_value = [1000, 0.0, 109.5, 0.0, 0.0, 1000, 0.0, 109.5, SubGimbal2DType]

			elif SubGimbal2DType == SUB_GIMBAL2D_TYPE.SUB_GIMBAL2D_TYPE_PWMTEST.value:
				self.gain_name = ['M1','M2','M3','M4','cmode']
				val = 65000
				self.gain_value = [val, val, val, val, SubGimbal2DType]

			if log_type == LOG_TYPE.LOG_TYPE_ANGPOS_TRQ.value:
				self.data_a_name = 'sctrl_Gimbal2D.alpha'
				self.data_c_name = 'sctrl_Gimbal2D.beta'
				self.data_b_name = 'sctrl_Gimbal2D.u_alpha'
				self.data_d_name = 'sctrl_Gimbal2D.u_beta'

			elif log_type == LOG_TYPE.LOG_TYPE_PWM_CMD.value:
				self.data_a_name = 'pm.vbat'
				self.data_b_name = 'sctrl_Gimbal2D.t_m2'
				self.data_c_name = 'sctrl_Gimbal2D.t_m3'
				self.data_d_name = 'sctrl_Gimbal2D.t_m4'

		elif controller_type == CONTROLLER_TYPE.CONTROLLER_TYPE_SINGLEPPID.value:
			self.group_name = 'sparam_ppid'
			self.config_name ='sctrl_ppid'
			self.set_group = 'sparam_ppid.{}'
			c1 = 3; c2 = 1
			self.gain_name = ['pgaina', 'igaina', 'dgaina', 'pgainb', 'igainb', 'dgainb', 
								'pgainas', 'igainas', 'dgainas', 'pgainbs', 'igainbs', 'dgainbs',
								's_tx', 's_ty', 's_tz']
			self.gain_value = [900, 15, 0, 1100, 9, 0, 
								0.00007/c1, 0.00006/c1, 0.000003/c1, 0.00004/c2, 0.00004/c2, 0.000002/c2,
								0.1,0.1,0.1]
			if log_type == LOG_TYPE.LOG_TYPE_ANGPOS_TRQ.value:
				self.data_a_name = 'sctrl_ppid.t_ae'
				self.data_b_name = 'sctrl_ppid.u_alpha'
				self.data_c_name = 'sctrl_ppid.t_be'
				self.data_d_name = 'sctrl_ppid.u_beta'
			elif log_type == LOG_TYPE.LOG_TYPE_PWM_CMD.value:
				self.data_a_name = 'sctrl_ppid.t_m1'
				self.data_b_name = 'sctrl_ppid.t_m2'
				self.data_c_name = 'sctrl_ppid.t_m3'
				self.data_d_name = 'sctrl_ppid.t_m4'

	def _connected(self, link_uri):

		print('Connected to %s' % link_uri)
		self._cf.param.add_update_callback(group=self.group_name, name=None, cb=self._param_callback)
		# modify the controller 
		self._cf.param.set_value('stabilizer.controller','{}'.format(ControllerType))
		self._cf.param.add_update_callback(group='stabilizer', name='controller', cb=self._stabilizer_controller_callback)

		# modify PID gains parameters
		for n in self.gain_name:
			ind = self.gain_name.index(n)
			self._cf.param.set_value(self.set_group.format(n), '{}'.format(self.gain_value[ind]))
			self._cf.param.remove_update_callback(group='controller_tune', name=n, cb=self._param_callback)

		# The definition of the logconfig can be made before connecting
		self._lg_battery = LogConfig(name='pm', period_in_ms=1000)
		self._lg_battery.add_variable('pm.vbat','float')
		self._lg_stab = LogConfig(name=self.config_name, period_in_ms=10)

		self._lg_stab.add_variable(self.data_a_name, 'float')
		self._lg_stab.add_variable(self.data_b_name, 'float')
		self._lg_stab.add_variable(self.data_c_name, 'float')
		self._lg_stab.add_variable(self.data_d_name, 'float')

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
  
	def _stabilizer_controller_callback(self, name, value):
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
		self.data_a = (data[self.data_a_name], 0)[math.isnan(data[self.data_a_name]) == True]
		self.data_b = (data[self.data_b_name], 0)[math.isnan(data[self.data_b_name]) == True]
		self.data_c = (data[self.data_c_name], 0)[math.isnan(data[self.data_c_name]) == True]
		self.data_d = (data[self.data_d_name], 0)[math.isnan(data[self.data_d_name]) == True]

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

	# # Initialize the low-level drivers (don't list the debug drivers)
	cflib.crtp.init_drivers(enable_debug_driver=False)
	# Scan for Crazyflies and use the first one found
	print('Scanning interfaces for Crazyflies...')
	# available = cflib.crtp.scan_interfaces()

	le = CrazyflieGimbal2D(ControlTarget, 0, ControllerType, LogType)
	time.sleep(0.25)
	logger = ab_logger(CMD_RATE, ControllerType, SubGimbal2DType, LogType, folder_name=DATA_FOLDER_NAME)
	time.sleep(4)

	cmd_rate = CMD_RATE # 200Hz
	t = 0

	if RefType == REF_TYPE.REF_TYPE_STEP.value:
		RefGen = StepReferenceGenerator(THRUST_CONST)
		T_final = 6
	elif RefType == REF_TYPE.REF_TYPE_RAMP.value:
		RefGen = TrajReferenceGenerator(THRUST_CONST)
		T_final = 20
	elif RefType == REF_TYPE.REF_TYPE_THRUST.value:
		RefGen = ThrustReferenceGenerator(THRUST_CONST)
		T_final = 10
	elif RefType == REF_TYPE.REF_TYPE_PWM.value:
		RefGen = ThrustReferenceGenerator(THRUST_CONST)
		T_final = 4

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
		logger.log_append(le.timest, RefGen.alpha, RefGen.beta, le.data_a, le.data_b, le.data_c, le.data_d)

	le._stop_crazyflie()
	RefGen.stop_controller = True
	logger.savelog()
	logger.plot(RefType, LogType)