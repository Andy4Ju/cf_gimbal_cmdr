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
	def __init__(self, Ts, ControllerType, SubControllerType, LogType, folder_name='log_files'):
		self.folder_name = folder_name
		self.Ts = Ts
		self.ControllerType = ControllerType
		self.SubControllerType = SubControllerType
		self.LogType = LogType
		if self.LogType == LOG_TYPE.LOG_TYPE_ANGPOS_TRQ.value:
			self.log_memory = [["timestamp", "alpha_d", "beta_d", "alpha_m", "beta_m", "u_alpha", "u_beta"]]
		elif self.LogType == LOG_TYPE.LOG_TYPE_PWM_CMD.value:
			self.log_memory = [["timestamp", "alpha_d", "beta_d", "cmd_m1", "cmd_m2", "cmd_m3", "cmd_m4"]]

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
			
		with open(self.folder_name + '/log_' + time.strftime("%m%d_%H%M%S_") + str(self.ControllerType) + str(self.SubControllerType) + str(self.LogType) + '.txt', 'w') as csvfile:
			writer = csv.writer(csvfile)
			writer.writerows(self.log_memory)
			print("CSV file: " + "log_" + time.strftime("%m%d_%H%M%S_") + str(self.ControllerType) + str(self.SubControllerType) + str(self.LogType) + ".txt" + " created")
			# ControllerType + SubControllerType + LogType (refer to parameter.py for more details!)

	def conv(self, s):
		try:
			s = float(s)
		except ValueError:
			pass
		return s
   
	def openCSVMatrix(self, filename = 'filename'):
		rows = []
		with open(filename) as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			for row in reader:
				newrow = []
				for cell in row:
					newcell = self.conv(cell)
					newrow.append(newcell)
				rows.append(newrow)
    
		rows_drop = list(filter(None, rows))
		return rows_drop

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
				plt.plot(timestamp, ad, 'r--', timestamp, da, 'k')
				plt.ylabel('angle (rad)')
				plt.title('alpha')
				plt.legend(['alpha_ref','alpha_fb'])
				plt.grid(True)
				plt.subplot(222)
				plt.plot(timestamp, bd, 'b--', timestamp, dc, 'k')
				plt.ylabel('angle (rad)')
				plt.title('beta')
				plt.legend(['beta_ref','beta_fb'])
				plt.grid(True)
			elif RefType == REF_TYPE.REF_TYPE_RAMP.value:
				alpha_error = ad - da
				beta_error = bd - dc
				plt.subplot(221)
				plt.plot(timestamp, ad, 'r--', timestamp, da, 'r', timestamp, alpha_error, 'k')
				plt.ylabel('angle (rad)')
				plt.title('alpha')
				plt.legend(['alpha_ref','alpha_fb','error'])
				plt.grid(True)

				plt.subplot(222)
				plt.plot(timestamp, bd, 'b--', timestamp, dc, 'b', timestamp, beta_error, 'k')
				plt.ylabel('angle (rad)')
				plt.title('beta')
				plt.legend(['beta_ref','beta_fb','error'])
				plt.grid(True)
			plt.subplot(223)
			plt.plot(timestamp, db, 'r')
			plt.ylabel('u alpha')
			plt.grid(True)
			plt.subplot(224)
			plt.plot(timestamp, dd, 'b')
			plt.ylabel('u beta')
			plt.grid(True)

		elif LogType == LOG_TYPE.LOG_TYPE_PWM_CMD.value: # Unit in Newton
			plt.subplot(221)
			plt.plot(timestamp, da, 'r')
			plt.ylabel('m1 (N)')
			plt.grid(True)
			plt.subplot(222)
			plt.plot(timestamp, db, 'g')
			plt.ylabel('m2 (N)')
			plt.grid(True)
			plt.subplot(223)
			plt.plot(timestamp, dc, 'b')
			plt.ylabel('m3 (N)')
			plt.grid(True)
			plt.subplot(224)
			plt.plot(timestamp, dd, 'm')
			plt.ylabel('m4 (N)')
			plt.grid(True)

		if RefType == REF_TYPE.REF_TYPE_STEP.value:
			[Alpha_Tr, Alpha_Mp, ATset] = eva.getStepInfo(ad, da, self.Ts)
			[Beta_Tr, Beta_Mp, BTset] = eva.getStepInfo(bd, dc, self.Ts)

			print('Alpha_Tr = %s seconds, Alpha_Mp = %s percent, ATset = %s' %(Alpha_Tr, Alpha_Mp, ATset))
			print('Beta_Tr = %s seconds, Beta_Mp = %s percent, BTset=%s' %(Beta_Tr, Beta_Mp, BTset))

		elif RefType == REF_TYPE.REF_TYPE_RAMP.value and LogType == LOG_TYPE.LOG_TYPE_ANGPOS_TRQ.value:
			# Alpha_error_std =  eva.getTrackingInfo(ad, da)
			# Beta_error_std =  eva.getTrackingInfo(bd, dc)
			# print('Alpha_error_std = %s, Beta_error_std = %s'%(Alpha_error_std, Beta_error_std))
			Alpha_error_rmse =  eva.getRMSInfo(ad, da, self.Ts)
			Beta_error_rmse =  eva.getRMSInfo(bd, dc, self.Ts)
			print('Alpha_error_rmse = %s, Beta_error_rmse = %s' %(Alpha_error_rmse, Beta_error_rmse))

		plt.show()