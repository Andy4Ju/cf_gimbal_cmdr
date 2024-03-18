import enum

class URL(enum.Enum):
	QC_GREY_ORANGE_URL = 'radio://0/80/2M/E7E7E7E7E8'
	QC_GREY_BROWN_URL = 'radio://0/120/2M/E7E7E7E7EF'
	QC_BLUE_BLUE_URL = 'radio://0/80/2M/E7E7E7E7E9'

class CONTROLLER(enum.Enum):
    CONTROLLER_GIMBAL2D = 1

class REFTYPE(enum.Enum):
    STEP = 1
    RAMP = 2