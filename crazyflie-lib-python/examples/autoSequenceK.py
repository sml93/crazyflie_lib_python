# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
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
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7AA'

# Change the sequence according to your setup
#             x    y    z  YAW
sequence1 = [
     (0, 0, 0.2, 0),
     (0, 0, 0.5, 0),
     (0, 0, 0.75, 0),
     (0, 0.25, 0.75, 0),
     (0, 0.5, 0.75, 0)
]

sequence2 = [
     (0, 0.5, 0.75, 0),
     (0, 0.25, 0.75, 0),
     (0, 0, 0.75, 0),
     (0, 0, 0.5, 0),
     (0, 0, 0.2, 0),
]

sequence3 = [
#     (0, 0, 0.1, 0),
     (0, 0, 0.2, 0),
     (0, 0, 0.2, 0),
     (0, 0, 0.2, 0),
     (0, 0, 0.2, 0),
     (0, 0, 0.2, 0),
     (0, 0, 0.2, 0),
#     (0, 0, 1.0, 0),
#     (0, 0, 1.2, 0),
#     (0, 0, 1.5, 0),
#     (0, 0, 1.1, 0),
#     (0, 0, 1.2, 0),
#     (0, 0, 1.5, 0),
]

sequence4 = [
#     (0, 0, 1.5, 0),
#     (0, 0, 1.2, 0),
#     (0, 0, 1.1, 0),
#     (0, 0, 1.5, 0),
#     (0, 0, 1.2, 0),
     (0, 0, 0.2, 0),
     (0, 0, 0.2, 0),
     (0, 0, 0.2, 0),
     (0, 0, 0.2, 0),
     (0, 0, 0.2, 0),
     (0, 0, 0.2, 0),
#     (0, 0, 0.2, 0),
#     (0, 0, 0.1, 0),
]



def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

###############################################################################

def run_sequence(scf, sequence):
    cf = scf.cf

    cf.param.set_value('flightmode.posSet', '1')

    for position in sequence:
        print('Setting position {}'.format(position))
        for i in range(20):
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
            time.sleep(0.1)


###############################################################################
def run_K(scf):
    cf = scf.cf
    r = 1.0
    botZ = 1.0
    midZ = 1.5
    topZ = 2.0
    steps_side = 20

    # From z = 0 to 2m
    for i in range(0, int(steps_side / 1) + 1):
        x = 0
        y = 0
        z = r * i / (steps_side / 2.0)
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '}')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.5)

    # From 2m to 1.5m
    r = 0.5
    for i in range(0, int(steps_side / 2) + 1):
        x = 0
        y = 0
        z_high = topZ - (r * i / (steps_side / 2.0))
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z_high, 3)) + '}')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.5)

    # From 1.5m mid to top K
    r = 1.0
    for i in range(0, int(steps_side / 2) + 1):
        x = 0
        y = r * i / (steps_side / 4.0)
        z = midZ + (y/4)
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '}')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.5)

    # From top K to 1.5m mid
    r = 1.0
    for i in range(0, int(steps_side / 2) + 1):
        x = 0
        y = 2.0 - r * i / (steps_side / 4.0)
        z = topZ - (r * i / (steps_side / 2.0)) / 2
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '}')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.5)

    # From 1.5m mid to bot K
    for i in range(0, int(steps_side / 2) + 1):
        x = 0
        y = r * i / (steps_side / 4.0)
        z = midZ - (y / 4)
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '}')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.5)

    # From bot K to 1.5m mid
    for i in range(0, int(steps_side / 2) + 1):
        x = 0
        y = 2.0 - r * i / (steps_side / 4.0)
        z = botZ + (r * i / (steps_side / 2.0)) / 2
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '}')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.5)

    # From z = 1.5m to 0m
    for i in range(0, int(steps_side / 2) + 1):
        x = 0
        y = 0
        z = midZ - r * i / (steps_side / 2.0)
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '}')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.5)
    cf.commander.send_position_setpoint(x,
                                        y,
                                        0.2,
                                        0)
    time.sleep(0.5)



###############################################################################
def run_square(scf, sequence):
    cf = scf.cf
    r = 0.5
    steps_side = 20

    ## TO SETPOINT 0 ##
    # From x = 0.0, y = 0.0 -> x = 0.0, y = 0.4
    print("\nEnroute to Setpoint 0")
    for i in range(0, int(steps_side / 2)):
        x = 0
        y = r * i / (steps_side / 2.0) #0,0.04,0.08,...,0.36
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: 0.75},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            1.5,
                                            0)
        time.sleep(0.1)
    cf.commander.send_position_setpoint(r, r, 1.5, 0)
    time.sleep(0.2)

    ## TO SETPOINT 1 ##
    # From x = 0.0, y = 0.4 -> x = y = 0.4
    print("\nEnroute to Setpoint 1")
    for i in range(0, int(steps_side / 2)):
        x = r * i / (steps_side / 2.0) #0,0.04,0.08,...,0.36
        y = r
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: 0.75},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            1.5,
                                            0)
        time.sleep(0.1)
    cf.commander.send_position_setpoint(r, r, 1.5, 0)
    time.sleep(0.2)
    print("7\t---\t0\t---\tx")
    print("|\t   \t \t   \t|")
    print("6\t   \tH\t   \t2")
    print("|\t   \t \t   \t|")
    print("5\t---\t4\t---\t3")


    ## TO SETPOINT 2 ##
    # From x = y = 0.4 -> x = 0.4, y = 0.0
    print("\nEnroute to Setpoint 2")
    for i in range(0, int(steps_side / 2)):
        x = r
        y = r + (r * (-i) / (steps_side / 2.0)) #0.4,0.36,0.32,...,0.04
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: 0.75},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            1.5,
                                            0)
        time.sleep(0.1)
    cf.commander.send_position_setpoint(r, 0.0, 1.5, 0)
    time.sleep(0.2)
    print("7\t---\t0\t---\t1")
    print("|\t   \t \t   \t|")
    print("6\t   \tH\t   \tx")
    print("|\t   \t \t   \t|")
    print("5\t---\t4\t---\t3")


    ## TO SETPOINT 3 ##
    # From x = 0.4, y = 0.0 -> x = 0.4, y = -0.4
    print("\nEnroute to Setpoint 3")
    for i in range(0, int(steps_side / 2)):
        x = r
        y = 0 + (r * (-i) / (steps_side / 2.0)) #0,-0.04,-0.08,...,-0.36
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: 0.75},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            1.5,
                                            0)
        time.sleep(0.1)
    cf.commander.send_position_setpoint(r, -r, 1.5, 0)
    time.sleep(0.2)
    print("7\t---\t0\t---\t1")
    print("|\t   \t \t   \t|")
    print("6\t   \tH\t   \t2")
    print("|\t   \t \t   \t|")
    print("5\t---\t4\t---\tx")


    ## TO SETPOINT 4 ##
    # From x = 0.4, y = -0.4 -> x = 0.0, y = -0.4
    print("\nEnroute to Setpoint 4")
    for i in range(0, int(steps_side / 2)):
        x = r + (r * (-i) / (steps_side / 2.0)) #0.4,0.36,0.32,...,0.04
        y = -r
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: 0.75},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            1.5,
                                            0)
        time.sleep(0.1)
    cf.commander.send_position_setpoint(0.0, -r, 1.5, 0)
    time.sleep(0.2)
    print("7\t---\t0\t---\t1")
    print("|\t   \t \t   \t|")
    print("6\t   \tH\t   \t2")
    print("|\t   \t \t   \t|")
    print("5\t---\tx\t---\t3")


    ## TO SETPOINT 5 ##
    # From x = 0.0, y = -0.4 -> x = -0.4, y = -0.4
    print("\nEnroute to Setpoint 5")
    for i in range(0, int(steps_side / 2)):
        x = 0 + (r * (-i) / (steps_side / 2.0)) #0,-0.04,0.08,...,-0.36
        y = -r
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: 0.75},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            1.5,
                                            0)
        time.sleep(0.1)
    cf.commander.send_position_setpoint(-r, -r, 1.5, 0)
    time.sleep(0.2)
    print("7\t---\t0\t---\t1")
    print("|\t   \t \t   \t|")
    print("6\t   \tH\t   \t2")
    print("|\t   \t \t   \t|")
    print("x\t---\t4\t---\t3")


    ## TO SETPOINT 6 ##
    # From x = -0.4, y = -0.4 -> x = -0.4, y = 0
    print("\nEnroute to Setpoint 6")
    for i in range(0, int(steps_side / 2)):
        x = -r
        y = -r + (r * (i) / (steps_side / 2.0)) #
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: 0.75},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            1.5,
                                            0)
        time.sleep(0.1)
    cf.commander.send_position_setpoint(-r, 0, 1.5, 0)
    time.sleep(0.2)
    print("7\t---\t0\t---\t1")
    print("|\t   \t \t   \t|")
    print("x\t   \tH\t   \t2")
    print("|\t   \t \t   \t|")
    print("5\t---\t4\t---\t3")


    ## TO SETPOINT 7 ##
    # From x = -0.4, y = 0 -> x = -0.4, y = 0.4
    print("\nEnroute to Setpoint 7")
    for i in range(0, int(steps_side / 2)):
        x = -r
        y = 0 + (r * (i) / (steps_side / 2.0)) #
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: 0.75},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            1.5,
                                            0)
        time.sleep(0.1)
    cf.commander.send_position_setpoint(-r, r, 1.5, 0)
    time.sleep(0.2)
    print("x\t---\t0\t---\t1")
    print("|\t   \t \t   \t|")
    print("6\t   \tH\t   \t2")
    print("|\t   \t \t   \t|")
    print("5\t---\t4\t---\t3")


    ## TO SETPOINT 0 ##
    # From x = -0.4, y = 0.4 -> x = 0, y = 0.4
    print("\nEnroute to Setpoint 0")
    for i in range(0, int(steps_side / 2)):
        x = -r + (r * (i) / (steps_side / 2.0))
        y = r
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: 0.75},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            1.5,
                                            0)
        time.sleep(0.1)
    cf.commander.send_position_setpoint(0, r, 1.5, 0)
    time.sleep(0.2)
    print("7\t---\tx\t---\t1")
    print("|\t   \t \t   \t|")
    print("6\t   \tH\t   \t2")
    print("|\t   \t \t   \t|")
    print("5\t---\t4\t---\t3")


###############################################################################

def hover(scf, sequence):
    cf = scf.cf
    h = 1.2
    steps_up = 10

    for i in range(0, steps_up):
        x = 0
        y = 0
        z = h * i / (steps_up)#0,0.12,0.24,...,1.08
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.5)
    cf.commander.send_position_setpoint(0, 0, h, 0)
    time.sleep(0.2)




def land(scf, sequence):
    cf = scf.cf
    h = 1.2
    steps_up = 10

    for i in range(0, steps_up):
        x = 0
        y = 0
        z = h + (-h * i / (steps_up))#1.2,1.08,0.96,...,0.12
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.5)
    cf.commander.send_position_setpoint(0, 0, 0, 0)
    time.sleep(0.2)




###############################################################################
###############################################################################

def run_surveillance(scf, sequence):
    cf = scf.cf
    r1 = 1.6
    r2 = 0.8
    steps_side = 20
    z = 1.2


    ## TO SETPOINT 0 ##
    # From x = 0.0, y = 0.0 -> x = 0.0, y = 0.5
    print("\nEnroute to Setpoint 0")
    for i in range(0, int(steps_side / 2)):
        x = 0
        y = r2 * i / (steps_side / 2.0) #0,0.05,0.1,...,0.45
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(0, r2, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 1 ##
    # From x = 0.0, y = 0.5 -> x = 1.0, y = 0.5
    print("\nEnroute to Setpoint 1")
    for i in range(0, int(steps_side)):
        x = r1 * i / (steps_side) #0,0.05,0.1,...,0.95
        y = r2
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(r1, r2, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 2 ##
    # From x = 1.0, y = 0.5 -> x = 1.0, y = 0.0
    print("\nEnroute to Setpoint 2")
    for i in range(0, int(steps_side / 2)):
        x = r1
        y = r2 + (r2 * (-i) / (steps_side / 2.0)) #0.5,0.45,0.4,...,0.05
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(r1, 0, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 3 ##
    # From x = 1.0, y = 0.0 -> x = 1.0, y = -0.5
    print("\nEnroute to Setpoint 3")
    for i in range(0, int(steps_side / 2)):
        x = r1
        y = (r2 * (-i) / (steps_side / 2.0)) #0,-0.05,-0.1,...,-0.45
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(r1, -r2, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 4 ##
    # From x = 1.0, y = -0.5 -> x = 0.5, y = -0.5
    print("\nEnroute to Setpoint 4")
    for i in range(0, int(steps_side / 2)):
        x = r1 - (r2 * i / (steps_side / 2.0)) #1.0,0.95,0.9,...,0.55
        y = -r2
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(r2, -r2, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 5 ##
    # From x = 0.5, y = -0.5 -> x = 0.5, y = 0
    print("\nEnroute to Setpoint 5")
    for i in range(0, int(steps_side / 2)):
        x = r2
        y = -r2 + (r2 * i / (steps_side / 2.0)) #-0.5,-0.45,-0.4,...,-0.05
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(r2, 0, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 6 ##
    # From x = 0.5, y = 0 -> x = 0.5, y = 0.5
    print("\nEnroute to Setpoint 6")
    for i in range(0, int(steps_side / 2)):
        x = r2
        y = (r2 * i / (steps_side / 2.0)) #0,0.05,0.1,...,0.45
        print('{x: ' + str(round(x,3)) + ', y: ' + str(round(y,3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(r2, r2, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 7 ##
    # From x = 0.5, y = 0.5 -> x = 0, y = 0.5
    print("\nEnroute to Setpoint 7")
    for i in range(0, int(steps_side / 2)):
        x = r2 + (-r2 * i / (steps_side / 2.0)) #0.5,0.45,0.4,...,0.05
        y = r2
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(0, r2, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 8 ##
    # From x = 0, y = 0.5 -> x = 0, y = 0
    print("\nEnroute to Setpoint 8")
    for i in range(0, int(steps_side / 2)):
        x = 0
        y = r2 + (-r2 * i / (steps_side / 2.0)) #0.5,0.45,0.4,...,0.05
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(0, 0, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 9 ##
    # From x = 0, y = 0 -> x = 0, y = -0.5
    print("\nEnroute to Setpoint 9")
    for i in range(0, int(steps_side / 2)):
        x = 0
        y = - (r2 * i / (steps_side / 2.0)) #-0,-0.5,-0.1,...,0.45
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(0, -r2, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 10 ##
    # From x = 0, y = -0.5 -> x = -0.5, y = -0.5
    print("\nEnroute to Setpoint 10")
    for i in range(0, int(steps_side / 2)):
        x = (-r2 * i / (steps_side / 2.0)) #0,-0.05,-0.1,...,-0.45
        y = -r2
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(-r2, -r2, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 11 ##
    # From x = -0.5, y = -0.5 -> x = -0.5, y = 0
    print("\nEnroute to Setpoint 11")
    for i in range(0, int(steps_side / 2)):
        x = -r2
        y = -r2 + (r2 * i / (steps_side / 2.0)) #-0.5,-0.45,0.4,...,0.05
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(-r2, 0, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 12 ##
    # From x = -0.5, y = 0 -> x = -0.5, y = 0.5
    print("\nEnroute to Setpoint 12")
    for i in range(0, int(steps_side / 2)):
        x = -r2
        y = (r2 * i / (steps_side / 2.0)) #0,0.05,0.1,...,0.45
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(-r2, r2, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 13 ##
    # From x = -0.5, y = 0.5 -> x = -1.0, y = 0.5
    print("\nEnroute to Setpoint 13")
    for i in range(0, int(steps_side / 2)):
        x = -r2 + (-r2 * i / (steps_side / 2.0)) #-0.5,-0.55,-0.6,...,-0.95
        y = r2
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(-r1, r2, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 14 ##
    # From x = -1.0, y = 0.5 -> x = -1.0, y = 0
    print("\nEnroute to Setpoint 14")
    for i in range(0, int(steps_side / 2)):
        x = -r1
        y = r2 + (-r2 * i / (steps_side / 2.0)) #0.5,0.45,0.4,...,0.05
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(-r1, 0, z, 0)
    time.sleep(0.2)



    ## TO SETPOINT 15 ##
    # From x = -1.0, y = 0 -> x = -1.0, y = -0.5
    print("\nEnroute to Setpoint 15")
    for i in range(0, int(steps_side / 2)):
        x = -r1
        y = (-r2 * i / (steps_side / 2.0)) #0,-0.05,-0.1,...,-0.45
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(-r1, -r2, z, 0)
    time.sleep(0.2)


    ## TO SETPOINT 16 ##
    # From x = -1.0, y = -0.5 -> x = 0.0, y = -0.5
    print("\nEnroute to Setpoint 16")
    for i in range(0, int(steps_side)):
        x = -r1 + (r1 * i / (steps_side)) #-1.0,-0.95,-0.9,...,-0.05
        y = -r2
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(0, -r2, z, 0)
    time.sleep(0.2)



    ## TO HOME ##
    # From x = 0.0, y = -0.5 -> x = 0.0, y = 0.0
    print("\nEnroute to Setpoint HOME")
    for i in range(0, int(steps_side / 2)):
        x = 0
        y = -r2 + (r2 * i / (steps_side / 2.0)) #-0.5,-0.45,-0.4,...,-0.05
        print('{x: ' + str(round(x, 3)) + ', y: ' + str(round(y, 3)) + ', z: ' + str(round(z, 3)) + '},')
        cf.commander.send_position_setpoint(x,
                                            y,
                                            z,
                                            0)
        time.sleep(0.15)
    cf.commander.send_position_setpoint(0, 0, z, 0)
    time.sleep(0.2)



### Main ###
if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        reset_estimator(scf)
        start_position_printing(scf)
        ## Flying to start point of square
#        print("\nEnroute to Setpoint 0")
#        run_sequence(scf, sequence1)
#        print("7\t---\tx\t---\t1")
#        print("|\t   \t \t   \t|")
#        print("6\t   \tH\t   \t2")
#        print("|\t   \t \t   \t|")
#        print("5\t---\t4\t---\t3")
        # run_sequence(scf, sequence3)
#        hover(scf, sequence3)
        run_K(scf)
        time.sleep(0.15)

##        ## Running sequence
#        for r in range(0,1):
#            run_surveillance(scf,sequence3)
#            time.sleep(0.2)

        ## RTL
#        print("\nEnroute to Home")
#        run_sequence(scf, sequence2)
#        print("7\t---\t0\t---\t1")
#        print("|\t   \t \t   \t|")
#        print("6\t   \tx\t   \t2")
#        print("|\t   \t \t   \t|")
#        print("5\t---\t4\t---\t3")
#        time.sleep(0.2)
        # run_sequence(scf, sequence4)
#        land(scf,sequence4)
        time.sleep(0.15)
        cf.commander.send_stop_setpoint()
