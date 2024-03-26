def pwm_test_ref(tnow):
	print('tnow = ', tnow)
	if tnow < 10:
		percent = 0.15
	else:
		percent = 0.3
		
	M1 = int(65535 * percent)
	M2 = int(65535 * percent)
	M3 = int(65535 * percent)
	M4 = int(65535 * percent)

	return[M1, M2, M3, M4]