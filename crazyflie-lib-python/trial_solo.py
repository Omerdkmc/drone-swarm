import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Perform varioous operations on the Crazyflie drones in a swarm
# such as taking off, landing, hovering, and running square sequences.

def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(2)

def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(1.25, 3.0)
    time.sleep(3)

def land(scf):
    commander= scf.cf.high_level_commander

    commander.land(0.0, 3.0)
    time.sleep(2)

    commander.stop()

def hover_sequence(scf):
    take_off(scf)
    land(scf)

def run_square_sequence(scf):
    box_size = 0.50
    flight_time = 2

    commander= scf.cf.high_level_commander

    commander.go_to(box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(-box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, -box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

def run_square_sequence_absolute(scf):

    flight_time = 2.0   # slower helps stability
    altitude    = 1.25 # match your take-off height
    sleep_time = 2.0
    sleep_time_in = 4.0 # time to wait before starting the sequence
    x_01, y_01 = 0.30, 0.30     # Position of the first drone 01 (x, y)
    square_edge = 1.0 # Length of the square's edge

    commander = scf.cf.high_level_commander

    # Position the first drone
    ################################################################
    commander.go_to(x_01, y_01, altitude, 0, flight_time, relative=False)
    time.sleep(sleep_time_in)


    ################################################################
    # First drone performs a square sequence
    ################################################################
    commander.go_to(x_01 + square_edge, y_01, altitude, 0, flight_time, relative=False)
    time.sleep(sleep_time)

    commander.go_to(x_01 + square_edge, y_01 + square_edge, altitude, 0, flight_time, relative=False)
    time.sleep(sleep_time)

    commander.go_to(x_01, y_01 + square_edge, altitude, 0, flight_time, relative=False)
    time.sleep(sleep_time)

    commander.go_to(x_01, y_01, altitude, 0, flight_time, relative=False)
    time.sleep(sleep_time)


def updown_funk(scf):

    flight_time = 2
    altitude = 1.0
    z_01 = 1.0
    x_01, y_01 = 1.0, 1.0
    delta_z = 0.75
    sleep_time = 1.5

    commander= scf.cf.high_level_commander

    commander.go_to(x_01, y_01, altitude, 0, flight_time, relative=False)
    time.sleep(2.0)

    # do 3 full up/down cycles
    for _ in range(4):

        commander.go_to(x_01, y_01, z_01 + delta_z, 0, flight_time, relative=False)
        time.sleep(sleep_time)
        commander.go_to(x_01, y_01, z_01, 0, flight_time, relative=False)
        time.sleep(sleep_time)
        commander.go_to(x_01, y_01, z_01-delta_z, 0, flight_time, relative=False)
        time.sleep(sleep_time)
        commander.go_to(x_01, y_01, z_01, 0, flight_time, relative=False)
        time.sleep(sleep_time)
        

def go_to_center(scf):
    altitude = 1.5
    flight_time = 3
    sleep_time = 10.0
    commander = scf.cf.high_level_commander
    commander.go_to(1.0, 1.0, altitude, 0, flight_time, relative=False)
    time.sleep(sleep_time)


def reset_estimator(scf):
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(2.0)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2.0)

uris = 'radio://0/20/2M/E7E7E7E701'
    # Add more URIs if you want more copters in the swarm
    # URIs in a swarm using the same radio must also be on the same channel


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with SyncCrazyflie(uris, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('Connected to  Crazyflies')
        # swarm.parallel_safe(light_check)
        # print('Resetting estimator')
        # reset_estimator(scf)
        # print('Estimators reset, arming motors!')

        take_off(scf)
        run_square_sequence_absolute(scf)
        # updown_funk(scf)
        land(scf)