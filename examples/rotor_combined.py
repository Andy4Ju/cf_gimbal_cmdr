# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects 2 Crazyflies, ramp up-down the motors and
disconnects.
"""
import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie

logging.basicConfig(level=logging.ERROR)

class combined_controller:
    """docstring for combined_controller"""
    def __init__(self):
        pass

    def step(self, states):
        thrust_mult = 1
        thrust_step = 500
        thrust = 20000
        pitch = 0
        roll = 0
        yawrate = 0

        return roll, pitch, yawrate, thrust




class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri, controller):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        self.connected = True

        self.qw = 0
        self.qx = 0
        self.qy = 0
        self.qz = 0

        self.controller = controller

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='State_Estimate', period_in_ms=10)
        self._lg_stab.add_variable('stateEstimate.qw', 'float')
        self._lg_stab.add_variable('stateEstimate.qx', 'float')
        self._lg_stab.add_variable('stateEstimate.qy', 'float')
        self._lg_stab.add_variable('stateEstimate.qz', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._scf.cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(_state_est_log_error)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(_state_est_log_data)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add State_Estimate log config, bad configuration.')

        # Start a timer to disconnect in 10s
        t = Timer(5, self._scf.cf.close_link)
        t.start()

        # Thread(target=self._ramp_motors).start()

        def _state_est_log_error(logconf, msg):
            """Callback from the log API when an error occurs"""
            print('Error when logging %s: %s' % (logconf.name, msg))

        def _state_est_log_data(timestamp, data, logconf):
            """Callback froma the log API when data arrives"""
            print('[%d][%s]: %s' % (timestamp, logconf.name, data))
            self.qw = data.qw
            self.qx = data.qx
            self.qy = data.qy
            self.qz = data.qz

            roll, pitch, yawrate, thrust = self.controller.step()
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))
        self.connected = False

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.connected = False





if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Connect the two Crazyflies and ramps them up-down
    le0 = MotorRampExample('radio://0/80/2M')
    le1 = MotorRampExample('radio://0/80/2M/E7E7E7E7E8')
    le2 = MotorRampExample('radio://0/80/2M/E7E7E7E7E9')
    le3 = MotorRampExample('radio://0/80/2M/E7E7E7E7EA')
    while(le0.connected or 
          le1.connected or
          le2.connected or
          le3.connected):
        time.sleep(0.1)
