import numpy as np

def getStepInfo(ref, fbk, Ts): # Ts is Command Time
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
		Tset_tick = 0
		Ymax = 0
		for y in range(t0_tick, tf_tick):
			if abs(fbk[y]) > abs(0.1 * Step_Value) and abs(fbk[y-1]) <= abs(0.1 * Step_Value) and Tr0_tick == 0:
				Tr0_tick = y
			if abs(fbk[y]) > abs(0.9 * Step_Value) and abs(fbk[y-1]) <= abs(0.9 * Step_Value) and Trf_tick == 0:
				Trf_tick = y
			if all(abs(fbk[y:tf_tick] - Step_Value) < 0.075 * Step_Value) and Tset_tick == 0:
				Tset_tick = y
			if abs(fbk[y]) > Ymax:
				Ymax = fbk[y]
		if Tr0_tick != 0 and Trf_tick != 0:
			Tr = (Trf_tick - Tr0_tick) * Ts
		else:
			Tr = -1
		if Tset_tick != 0:
			Tset = (Tset_tick - t0_tick) * Ts
		else:
			Tset = -1
		Mp = (Ymax / Step_Value - 1) * 100
	else:
		Tr = 0
		Mp = 0
		Tset = 0

	# print('Tr0_tick = %s, Trf_tick = %s'%(Tr0_tick, Trf_tick))
	# print('Ymax = ',Ymax)
	return [Tr, Mp, Tset]

def getTrackingInfo(ref, fbk):
	error = ref - fbk
	return np.std(error)

def getRMSInfo(ref, fbk, Ts): # Ts is Sampling Time
	l = len(ref)
	Signal_flag = 0 # Verify that Signal comes in
	for x in range(1, l):
		if ref[x] != 0 and ref[x-1] == 0:
			t0_tick = x
		if ref[x] == 0 and ref[x-1] != 0:
			tf_tick = x
			Signal_flag = 1
   
	if Signal_flag: # Calculate RMSE only for Tracking Period
		data_ref = ref[t0_tick:tf_tick]
		data_fbk = fbk[t0_tick:tf_tick]
		squared_errors = (data_ref - data_fbk) ** 2
		rmse = np.sqrt(np.mean(squared_errors))
	else:
		rmse = -1
		print('No Signal Detected?')
	
	return rmse