# Mike Wen 2023-09-29
# https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_connect_log_param/
# 連接/讀取資料/異步讀取/讀寫參數
# The maximum length for a log packet is 26 bytes. This, for for example, allows to log 6 floats and one uint16_t (6*4 + 2 bytes) in a single packet.
# The minimum period of a for a log configuration is multiples of 10ms

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.utils import uri_helper

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# The cflib.crtp module is for scanning for Crazyflies instances.
# The Crazyflie class is used to easily connect/send/receive data from a Crazyflie.
# The synCrazyflie class is a wrapper around the “normal” Crazyflie class. 
# It handles the asynchronous nature of the Crazyflie API and turns it into blocking function.
# LogConfig class is a representation of one log configuration that enables logging from the Crazyflie
# The SyncLogger class provides synchronous access to log data from the Crazyflie.

# uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

#This is the radio uri of the crazyflie, i
# t can be set by setting the environment variable CFLIB_URI, 
# if not set it uses the default. 
# It should be probably fine but if you do not know what the uri of your Crazyfie is you can check that with an usb cable and looking at the config (here are the instructions)

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()

# Here you add the logging configuration to the logging framework of the Crazyflie. 
# It will check if the log configuration is part of the TOC, 
# which is a list of all the logging variables defined in the Crazyflie. 
# You can test this out by changing one of the lg_stab variables to a completely bogus name like 'not.real'. 
# In this case you would receive the following message: KeyError: 'Variable not.real not in TOC'

def param_stab_est_callback(name, value):
    print('The crazyfile has parameter ' + name + ' set at number: ' + value)

def simple_param_async(scf, groupstr, namestr):
    cf = scf.cf
    full_name = groupstr + "." + namestr
    cf.param.add_update_callback(group=groupstr, name=namestr, cb=param_stab_est_callback)
    time.sleep(1)
    cf.param.set_value(full_name,2)
    time.sleep(1)
    cf.param.set_value(full_name,1)
    time.sleep(1)
    # The sleep function is to give the script a bit more time to wait for the Crazyflies response and not lose the connection immediately.

def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log(scf, logconf):
    with SyncLogger(scf, lg_stab) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s'%(timestamp, logconf_name, data))

            break

def simple_connect():

    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='stateEstimate',period_in_ms=10)

    # Here you will add the logs variables you would like to read out.
    # Variable name checked by connecting to Crazyflie to the cfclient and look at the log TOC tab
    lg_stab.add_variable('stateEstimate.x','float')
    lg_stab.add_variable('stateEstimate.y','float')
    lg_stab.add_variable('stateEstimate.z','float')

    lg_stab.add_variable('stateEstimate.qx', 'float')
    lg_stab.add_variable('stateEstimate.qy', 'float')
    lg_stab.add_variable('stateEstimate.qz', 'float')
    lg_stab.add_variable('stateEstimate.qw', 'float')      

    group = "stateEstimate"
    name = "estimator"    

    with SyncCrazyflie('radio://0/80/2M/E7E7E7E7E7', cf=Crazyflie(rw_cache='./cache')) as scf:

        simple_connect()
        #simple_log_async(scf, lg_stab)
        #simple_param_async(scf, group, name)

#The syncCrazyflie will create a synchronous Crazyflie instance with the specified link_uri. 
# As you can see we are currently calling an non-existing function, 
# so you will need to make that function first before you run the script.