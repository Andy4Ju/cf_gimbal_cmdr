import optirx as rx
import time, math, threading

class Optitrack():
	"""docstring for optitrack"""
	def __init__(self, debug=False):
		self.dsock = rx.mkdatasock(ip_address='192.168.2.136')
		self.version = (2, 10, 0, 0)  # NatNet version to use
		self.position = (0,0,0)
		self.quaternion = (0,0,0,1)
		self.lock = threading.Lock()
		self.debug = debug

		thr = threading.Thread(target=self.get_data, args=())
		thr.start()
		# thr.join()

	def get_data(self):
		while True:
			data = self.dsock.recv(rx.MAX_PACKETSIZE)
			packet = rx.unpack(data, version=self.version)
			if type(packet) is rx.SenderData:
				self.version = packet.natnet_version
			with self.lock:
				self.position = self.postition_enu2ned(packet.rigid_bodies[0].position)
				self.orientation = self.orientation_enu2ned(packet.rigid_bodies[0].orientation)

			if self.debug is True:
				print packet.rigid_bodies[0].position, packet.rigid_bodies[0].orientation
				# print self.orientation_enu2ned(packet.rigid_bodies[0].orientation)
				# print self.quaternion_to_euler_angle((packet.rigid_bodies[0].orientation))

			time.sleep(.03)


	def quaternion_to_euler_angle(self, q):
		y, x, z, w = q
		z = -z
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		X = math.degrees(math.atan2(t0, t1))
		
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		Y = math.degrees(math.asin(t2))
		
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		Z = math.degrees(math.atan2(t3, t4))
		
		return X, Y, Z

	def postition_enu2ned(self, position):
		return position[1], position[0], -position[2]

	def orientation_enu2ned(self, quaternion):
		return quaternion[3], quaternion[1], quaternion[0], -quaternion[2] # w, y, x, -z


