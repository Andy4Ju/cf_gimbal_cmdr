import numpy as np

def getStepInfo(ref, fbk, Ts):
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

def getTrackingInfo(ref, fbk):
	error = ref - fbk
	return np.std(error)