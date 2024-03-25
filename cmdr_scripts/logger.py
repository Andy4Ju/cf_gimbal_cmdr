import os
import matplotlib.pyplot as plt
import csv
import time
import numpy as np
import evaluateFbkInfo as eva
from parameter import REF_TYPE, LOG_TYPE

class ab_logger:
	"""
	logger class. record data and export to a csv file
	"""
	def __init__(self, Ts, ControllerType, SubControllerType, folder_name='log_files'):
		self.folder_name = folder_name
		self.log_memory = [["timestamp", "alpha_d", "beta_d", "data_a", "data_b", "data_c", "data_d"]]
		self.Ts = Ts
		self.ControllerType = ControllerType
		self.SubControllerType = SubControllerType

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
			
		with open(self.folder_name + '/log_' + time.strftime("%m%d_%H%M%S_") + str(self.ControllerType) + str(self.SubControllerType) + '.txt', 'w') as csvfile:
			writer = csv.writer(csvfile)
			writer.writerows(self.log_memory)
			print("CSV file: " + "log_" + time.strftime("%m%d_%H%M%S_") + str(self.ControllerType) + str(self.SubControllerType) + ".txt" + " created")

	def plot(self, RefType, LogType):
		n = len(self.log_memory)
		log_memory_array = np.asarray(self.log_memory[1:n])
		timestamp = (log_memory_array[:, 0] - log_memory_array[0, 0]) * 0.001
		ad = log_memory_array[:, 1]
		bd = log_memory_array[:, 2]
		da = log_memory_array[:, 3]
		db = log_memory_array[:, 4]
		dc = log_memory_array[:, 5]
		dd = log_memory_array[:, 6]

		if LogType == LOG_TYPE.LOG_TYPE_ANGPOS_TRQ.value:
			if RefType == REF_TYPE.REF_TYPE_STEP.value:
				plt.subplot(221)
				plt.plot(timestamp, ad, 'b--', timestamp, da, 'k')
				plt.ylabel('angle (rad)')
				plt.title('alpha')
				plt.legend(['alpha_ref','alpha'])
				plt.grid(True)
				plt.subplot(222)
				plt.plot(timestamp, bd, 'b--', timestamp, dc, 'k')
				plt.ylabel('angle (rad)')
				plt.title('beta')
				plt.legend(['beta_ref','beta'])
				plt.grid(True)
			elif RefType == REF_TYPE.REF_TYPE_RAMP.value:
				alpha_error = ad - da
				beta_error = bd - dc
				plt.subplot(221)
				plt.plot(timestamp, ad, 'b--', timestamp, da, 'k', timestamp, alpha_error, 'r')
				plt.ylabel('angle (rad)')
				plt.title('alpha')
				plt.legend(['alpha_ref','alpha','error'])
				plt.grid(True)

				plt.subplot(222)
				plt.plot(timestamp, bd, 'b--', timestamp, dc, 'k', timestamp, beta_error, 'r')
				plt.ylabel('angle (rad)')
				plt.title('beta')
				plt.legend(['beta_ref','beta','error'])
				plt.grid(True)
			plt.subplot(223)
			plt.plot(timestamp, db, 'b')
			plt.ylabel('u alpha')
			plt.grid(True)
			plt.subplot(224)
			plt.plot(timestamp, dd, 'b')
			plt.ylabel('u beta')
			plt.grid(True)

		elif LogType == LOG_TYPE.LOG_TYPE_PWM_CMD.value:
			plt.subplot(221)
			plt.plot(timestamp, da, 'b')
			plt.ylabel('m1_PWM')
			plt.grid(True)
			plt.subplot(222)
			plt.plot(timestamp, db, 'b')
			plt.ylabel('m2_PWM')
			plt.grid(True)
			plt.subplot(223)
			plt.plot(timestamp, dc, 'b')
			plt.ylabel('m3_PWM')
			plt.grid(True)
			plt.subplot(224)
			plt.plot(timestamp, dd, 'b')
			plt.ylabel('m4_PWM')
			plt.grid(True)

		if RefType == REF_TYPE.REF_TYPE_STEP.value:
			[Alpha_Tr, Alpha_Mp, ATset] = eva.getStepInfo(ad, da, self.Ts)
			[Beta_Tr, Beta_Mp, BTset] = eva.getStepInfo(bd, dc, self.Ts)

			print('Alpha_Tr = %s seconds, Alpha_Mp = %s percent, ATset = %s'%(Alpha_Tr, Alpha_Mp, ATset))
			print('Beta_Tr = %s seconds, Beta_Mp = %s percent, BTset=%s'%(Beta_Tr, Beta_Mp, BTset))

		elif RefType == REF_TYPE.REF_TYPE_RAMP.value:
			Alpha_error_std =  eva.getTrackingInfo(ad, da)
			Beta_error_std =  eva.getTrackingInfo(bd, dc)
			print('Alpha_error_std = %s, Beta_error_std = %s'%(Alpha_error_std, Beta_error_std))

		plt.show()