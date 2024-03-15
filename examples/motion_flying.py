# Mike Wen 2023-09-29
# https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_motion_commander/
# radio://0/10/2M : Radio interface, USB dongle number 0, radio channel 10 and radio speed 2 Mbit/s: radio://0/10/2M
# debug://0/1 : Debug interface, id 0, channel 1
# usb://0 : USB cable to microusb port, id 0
# serial://ttyAMA0 : Serial port, id ttyAMA0
# tcp://aideck-AABBCCDD.local:5000 : TCP network connection, Name: aideck-AABBCCDD.local, port 5000

import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

#Uniform Resource Identifier, radio interface, USB dongle number 0, radio channel 80, radio speed 2Mbit/s
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()

if __name__ == '__main__':

    cflib.crtp.init_drivers()
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # check if Flow deck is installed
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        take_off_simple(scf)
                
        time.sleep(1)
