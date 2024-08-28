import time
import numpy as np

class StepReferenceGenerator:
	def __init__(self, thrust):
		self.alpha = 0
		self.beta = 0
		self.thrust = 0 # 42598
		self.thrust_constant = thrust
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
				self.test_step(tnow)
				last_loop_time = current_time
			else:
				time.sleep(0.002)
    
	def test_step(self, tnow):
		# print('tnow = ', tnow)
		if tnow < 0:
			self.alpha = 0
			self.beta = 0
			self.thrust = 0
		elif tnow < 1:
			self.alpha = 0
			self.beta = 0
			self.thrust = self.thrust_constant
		elif tnow < 3:
			self.alpha = 0.5
			self.beta = 0
			self.thrust = self.thrust_constant	
		elif tnow < 4:
			self.alpha = 0
			self.beta = 0
			self.thrust = self.thrust_constant
		elif tnow < 6:
			self.alpha = 0
			self.beta = 0.5
			self.thrust = self.thrust_constant
		elif tnow < 7:
			self.alpha = 0
			self.beta = 0
			self.thrust = self.thrust_constant
		else:
			self.alpha = 0; self.beta = 0; self.thrust = 0

class TrajReferenceGenerator:
	def __init__(self, thrust):
		self.alpha = 0
		self.beta = 0
		self.thrust = 0 # 42598
		self.thrust_constant = thrust
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
				self.test_betalpha(tnow)
				last_loop_time = current_time
			else:
				time.sleep(0.002)
	
	def test_betalpha(self, tnow):
		# print('tnow = ', tnow)
		if tnow < 3:
			self.alpha = 0
			self.beta = 0
			self.thrust = self.thrust_constant*tnow/3
		elif tnow < 6:
			self.alpha = 0
			self.beta = np.pi/4*(tnow-3)/3
			self.thrust = self.thrust_constant
		elif tnow < 9:
			self.alpha = np.pi/6*(tnow-6)/3
			self.beta = np.pi/4
			self.thrust = self.thrust_constant	
		elif tnow < 12:
			self.alpha = np.pi/6
			self.beta = np.pi/4
			self.thrust = self.thrust_constant
		elif tnow < 15:
			self.alpha = np.pi/6
			self.beta = np.pi/4*(1-(tnow-12)/3)
			self.thrust = self.thrust_constant
		elif tnow < 18:
			self.alpha = np.pi/6*(1-(tnow-15)/3)
			self.beta = 0
			self.thrust = self.thrust_constant
		else:
			self.alpha = 0; self.beta = 0; self.thrust = 0
   
class ThrustReferenceGenerator:
	def __init__(self, thrust):
		self.alpha = 0
		self.beta = 0
		self.thrust = 0 # 42598
		self.thrust_constant = thrust
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
		# print('tnow = ', tnow)
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
			# self.thrust = self.thrust_constant*(1-(tnow-6)/3)
			self.thrust = self.thrust_constant
		else:
			self.alpha = 0; self.beta = 0; self.thrust = 0

class G3DReferenceGenerator:
	def __init__(self, thrust):
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.thrust = 0 # 42598
		self.thrust_constant = thrust
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
				self.test_G3D(tnow)
				last_loop_time = current_time
			else:
				time.sleep(0.002)
    
	def test_G3D(self, tnow):
		# print('tnow = ', tnow)
		if tnow < 3:
			self.roll = 0
			self.pitch = 0
			self.yaw = 0
			self.thrust = self.thrust_constant*tnow/3
		elif tnow < 5:
			self.roll = 0
			self.pitch = 0.8
			self.yaw = 0
			self.thrust = self.thrust_constant
		elif tnow < 6:
			self.roll = 0
			self.pitch = 0
			self.yaw = 0
			self.thrust = self.thrust_constant
		elif tnow < 8:
			self.roll = 0.6
			self.pitch = 0
			self.yaw = 0
			self.thrust = self.thrust_constant
		elif tnow < 9.5:
			self.roll = 0
			self.pitch = 0
			self.yaw = 0
			self.thrust = self.thrust_constant
		else:
			self.roll = 0; self.pitch = 0; self.yaw = 0; self.thrust = 0