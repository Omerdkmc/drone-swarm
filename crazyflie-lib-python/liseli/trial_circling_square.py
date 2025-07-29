# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2023 Bitcraze AB
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
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple "show" example that connects to 8 crazyflies (check the addresses at the top
and update it to your crazyflies addresses) and uses the high level commander with bezier curves
to send trajectories to fly a circle (while the 8 drones are positioned in a square).
To spice it up, the LEDs are changing color - the color move factor defines how fast and in which direction.

This example is intended to work with any positioning system (including LPS).
It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints using the high level commander.
"""
import sys
import time

import numpy as np

import cflib.crtp
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.crazyflie.mem import CompressedSegment
from cflib.crazyflie.mem import CompressedStart
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import glob
import pandas as pd

URI1 = 'radio://0/20/2M/E7E7E7E705'
URI2 = 'radio://0/20/2M/E7E7E7E706'
URI3 = 'radio://1/60/2M/E7E7E7E707'
URI4 = 'radio://1/60/2M/E7E7E7E708'
# URI5 = 'radio://0/60/2M/E7E7E7E714'
# URI6 = 'radio://0/60/2M/E7E7E7E715'
# URI7 = 'radio://0/60/2M/E7E7E7E716'
# URI8 = 'radio://0/60/2M/E7E7E7E717'

# The trajectory to fly
a = 0.55  # where the Beizer curve control point should be https://spencermortensen.com/articles/bezier-circle/
h = 1.0  # [m] how high we should fly
t = 4.0  # seconds per step, one circle has 4 steps
r1 = 1.75  # [m] the radius for all drones
r2 = 1.75  # [m] keep consistent, not used
center_x = 2.05
center_y = 2.4
loops = 5  # how many loops we should fly
color_move_factor = 3  # magic factor which defines how fast the colors move


def rotate_beizer_node(xl, yl, alpha):
    x_rot = []
    y_rot = []
    for x, y in zip(xl, yl):
        x_rot.append(x*np.cos(alpha) - y*np.sin(alpha))
        y_rot.append(x*np.sin(alpha) + y*np.cos(alpha))
    return x_rot, y_rot


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')


def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    trajectory_mem.trajectory = trajectory

    upload_result = trajectory_mem.write_data_sync()
    if not upload_result:
        print('Upload failed, aborting!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(
        trajectory_id,
        0,
        len(trajectory),
        type=HighLevelCommander.TRAJECTORY_TYPE_POLY4D_COMPRESSED)

    total_duration = 0
    # Skip the start element
    for segment in trajectory[1:]:
        total_duration += segment.duration

    return total_duration


def turn_off_leds(scf):
    # Set solid color effect
    scf.cf.param.set_value('ring.effect', '7')
    # Set the RGB values
    scf.cf.param.set_value('ring.solidRed', '0')
    scf.cf.param.set_value('ring.solidGreen', '0')
    scf.cf.param.set_value('ring.solidBlue', '0')


def run_sequence(scf, alpha, r):
    # Arm the Crazyflie
    scf.cf.platform.send_arming_request(True)
    time.sleep(1.0)

    commander = scf.cf.high_level_commander
    trajectory_id = 1
    duration = 4*t
    commander.takeoff(h, 2.0)
    time.sleep(3.0)
    x_start, y_start = rotate_beizer_node([r], [0.0], alpha)
    # Offset by center_x, center_y to match trajectory
    x0 = x_start[0] + center_x
    y0 = y_start[0] + center_y
    commander.go_to(x0, y0, h, 0.0, 2.0)
    time.sleep(3.0)
    relative = False
    start_time_leds = time.time()
    for i in range(loops):
        commander.start_trajectory(trajectory_id, 1.0, relative)
        time.sleep(duration)  # Wait for trajectory to finish before next loop or landing
    # Smooth landing instead of direct land
    smooth_land(commander, x0, y0)
    time.sleep(2.0)  # sleep long enough to be sure to have turned off leds
    commander.stop()


def create_trajectory(alpha, r):
    x_start, y_start = rotate_beizer_node([r], [0.0], alpha)
    beizer_point_1_x, beizer_point_1_y = rotate_beizer_node([r, r*a, 0.0], [r*a, r, r], alpha)
    beizer_point_2_x, beizer_point_2_y = rotate_beizer_node([-r*a, -r, -r], [r, r*a, 0.0], alpha)
    beizer_point_3_x, beizer_point_3_y = rotate_beizer_node([-r, -r*a, 0.0], [-r*a, -r, -r], alpha)
    beizer_point_4_x, beizer_point_4_y = rotate_beizer_node([r*a, r, r], [-r, -r*a, 0.0], alpha)

    # Offset all points by center_x, center_y
    x_start = [x + center_x for x in x_start]
    y_start = [y + center_y for y in y_start]
    beizer_point_1_x = [x + center_x for x in beizer_point_1_x]
    beizer_point_1_y = [y + center_y for y in beizer_point_1_y]
    beizer_point_2_x = [x + center_x for x in beizer_point_2_x]
    beizer_point_2_y = [y + center_y for y in beizer_point_2_y]
    beizer_point_3_x = [x + center_x for x in beizer_point_3_x]
    beizer_point_3_y = [y + center_y for y in beizer_point_3_y]
    beizer_point_4_x = [x + center_x for x in beizer_point_4_x]
    beizer_point_4_y = [y + center_y for y in beizer_point_4_y]

    trajectory = [
        CompressedStart(x_start[0], y_start[0], h, 0.0),
        CompressedSegment(t, beizer_point_1_x, beizer_point_1_y, [h], []),
        CompressedSegment(t, beizer_point_2_x, beizer_point_2_y, [h], []),
        CompressedSegment(t, beizer_point_3_x, beizer_point_3_y, [h], []),
        CompressedSegment(t, beizer_point_4_x, beizer_point_4_y, [h], []),
    ]
    return trajectory

# Smoother landing function
def smooth_land(commander, x, y):
    # Gradually descend before final land
    commander.go_to(x, y, 0.5, 0.0, 2.0)
    time.sleep(2.0)
    commander.go_to(x, y, 0.2, 0.0, 2.0)
    time.sleep(1.5)
    commander.land(0.0, 3.0)


def upload_trajectories(scf, alpha, r):
    trajectory_id = 1
    trajectory = create_trajectory(alpha, r)
    upload_trajectory(scf.cf, trajectory_id, trajectory)


def position_callback_factory(filename):
    def position_callback(timestamp, data, logconf):
        x = data['kalman.stateX']
        y = data['kalman.stateY']
        z = data['kalman.stateZ']
        with open(filename, 'a') as f:
            f.write('{},{},{},{}\n'.format(timestamp, x, y, z))
    return position_callback

def start_position_printing(scf, filename):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback_factory(filename))
    log_conf.start()
    return log_conf



if __name__ == '__main__':
    cflib.crtp.init_drivers()
    uris = [URI1, URI2, URI3, URI4]
    position_params = {
        URI1: [0.0, r1],           # 0° (East)
        URI2: [np.pi/2, r1],       # 90° (North)
        URI3: [np.pi, r1],         # 180° (West)
        URI4: [3*np.pi/2, r1]      # 270° (South)
    }

    # Remove old log files if they exist
    for uri in uris:
        drone_id = uri[-2:]
        log_path = f"C:/Users/omerr/drone-swarm/crazyflie-lib-python/liseli/data_circling_square_{drone_id}.csv"
        try:
            import os
            if os.path.exists(log_path):
                os.remove(log_path)
        except Exception as e:
            print(f"Could not remove log file {log_path}: {e}")

    # Prepare log_args for each drone
    log_args = {
        uri: (f"C:/Users/omerr/drone-swarm/crazyflie-lib-python/liseli/data_circling_square_{uri[-2:]}.csv",)
        for uri in uris
    }

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        print("Swarm created, connecting to Crazyflies...")
        print("Resetting estimators...")
        swarm.reset_estimators()
        time.sleep(1)
        print('Estimators reset..')
        # swarm.parallel_safe(turn_off_leds)
        print("Activating high level commander...")
        swarm.parallel_safe(upload_trajectories, args_dict=position_params)
        time.sleep(5)
        print("Starting position logging...")
        swarm.parallel_safe(start_position_printing, args_dict=log_args)
        time.sleep(1)
        swarm.parallel_safe(run_sequence, args_dict=position_params)



