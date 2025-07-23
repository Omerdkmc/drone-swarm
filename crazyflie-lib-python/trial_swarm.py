import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm


# Perform varioous operations on the Crazyflie drones in a swarm
# such as taking off, landing, hovering, and running square sequences.



##### Activate and deactivate LEDs #####
def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(2)

####################################




#### Take off and land functions ####
def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(1.5, 3.0)
    time.sleep(3)

def land(scf):
    commander= scf.cf.high_level_commander

    commander.land(0.0, 6.0)
    time.sleep(4)

    commander.stop()

####################################



#### Hover and land function ####
def hover_sequence(scf):
    take_off(scf)
    land(scf)

####################################



#### Up and down function for 2 drones ####
def updown_funk(scf):
    box_size = 0.50
    flight_time = 2

    commander= scf.cf.high_level_commander

    # decide who starts going up
    uri = scf.cf.link_uri
    up_first = uri.endswith('E7E7E7E702')  # True for CF1, False for CF2

    # do 3 full up and down cycles
    for i in range(4):


        if i == 1:
            if up_first:
                commander.go_to(0, 0,  box_size, 0, flight_time, relative=False)
            else:
                commander.go_to(0, 0, -box_size, 0, flight_time, relative=False)
            time.sleep(flight_time)

        else:

            if up_first:
                commander.go_to(0, 0, -2*box_size, 0, flight_time, relative=False)
            else:
                commander.go_to(0, 0,  2*box_size, 0, flight_time, relative=False)
            time.sleep(flight_time)

            if up_first:
                commander.go_to(0, 0,  2*box_size, 0, flight_time, relative=False)
            else:
                commander.go_to(0, 0, -2*box_size, 0, flight_time, relative=False)
            time.sleep(flight_time)

####################################



#### Run square sequence. The drones perform square trajectory on RELATIVE positions ####
def run_square_sequence(scf):
    box_size = 0.75
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

#####################################

def square_motion(scf):
    flight_time = 2.0   # slower helps stability
    altitude    = 1.5 # match your take-off height
    sleep_time = 3.0

    
    x_01, y_01 = 0.50, 0.50    # Position of the drone1 drone 01 (x, y). Drone moves to -> (x_01, y_01)
    x_02, y_02 = 2.0, 0.50    # Position of the second drone 02 (x, y). Drone moves to -> (x_02, y_02)
    x_03, y_03 = 0.50, 2.0    # Position of the first drone 03 (x, y). Drone moves to -> (x_01, y_01)
    x_04, y_04 = 2.0, 2.0

    commander = scf.cf.high_level_commander
    uri = scf.cf.link_uri
    drone1 = uri.endswith('E7E7E7E701')
    drone3 = uri.endswith('E7E7E7E703')
    drone4 = uri.endswith('E7E7E7E704')
    drone6 = uri.endswith('E7E7E7E706')


    if drone1:

        # Position the first drone
        ################################################################
        commander.go_to(x_01, y_01, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)


        ################################################################
        # First drone performs a square sequence
        ################################################################
        commander.go_to(x_02, y_02, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_04, y_04, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_03, y_03, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_01, y_01, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

    elif drone3:

        # Position the second drone
        ################################################################
        commander.go_to(x_02, y_02, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

       
        ################################################################
        # Second drone performs a square sequence
        ################################################################
        commander.go_to(x_04, y_04, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_03, y_03, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_01, y_01, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_02, y_02, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)



    elif drone6:

        # Position the second drone
        ################################################################
        commander.go_to(x_03, y_03, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

       
        ################################################################
        # Second drone performs a square sequence
        ################################################################
        commander.go_to(x_01, y_01, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_02, y_02, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_04, y_04, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_03, y_03, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)
    
    elif drone4:

        # Position the second drone
        ################################################################
        commander.go_to(x_04, y_04, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

       
        ################################################################
        # Second drone performs a square sequence
        ################################################################

        commander.go_to(x_03, y_03, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_01, y_01, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_02, y_02, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_04, y_04, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

    time.sleep(2)  # Wait for the drones to settle down after the sequence

   
    


#### Run square sequence with absolute positions ####
def run_square_sequence_absolute(scf):

    flight_time = 2.0   # slower helps stability
    altitude    = 1.5 # match your take-off height
    sleep_time = 4.0

    commander = scf.cf.high_level_commander
    uri = scf.cf.link_uri
    drone1 = uri.endswith('E7E7E7E701')
    drone2 = uri.endswith('E7E7E7E702')
    drone3 = uri.endswith('E7E7E7E703')
    drone4 = uri.endswith('E7E7E7E704')

    x_01, y_01 = 0.50, 0.50    # Position of the drone1 drone 01 (x, y). Drone moves to -> (x_01, y_01)
    x_02, y_02 = 1.5, 0.50    # Position of the second drone 02 (x, y). Drone moves to -> (x_02, y_02)
    x_03, y_03 = 1.5, 1.5    # Position of the first drone 03 (x, y). Drone moves to -> (x_01, y_01)
    x_04, y_04 = 0.5, 1.5
    square_edge = 0.60         # Length of the square's edge

    if drone1:

        # Position the first drone
        ################################################################
        commander.go_to(x_01, y_01, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)


        ################################################################
        # First drone performs a square sequence
        ################################################################
        commander.go_to(x_01 + square_edge, y_01, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

        commander.go_to(x_01 + square_edge, y_01 + square_edge, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_01, y_01 + square_edge, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_01, y_01, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

    elif drone2:

        # Position the second drone
        ################################################################
        commander.go_to(x_02, y_02, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

       
        ################################################################
        # Second drone performs a square sequence
        ################################################################
        commander.go_to(x_02 + square_edge, y_02, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_02 + square_edge, y_02 + square_edge, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_02, y_02 + square_edge, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_02, y_02, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

    elif drone3:

        # Position the second drone
        ################################################################
        commander.go_to(x_03, y_03, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

       
        ################################################################
        # Second drone performs a square sequence
        ################################################################
        commander.go_to(x_03 - square_edge, y_03, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_03 - square_edge, y_03 - square_edge, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_03, y_03 - square_edge, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_03, y_03, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)
    
    elif drone4:

        # Position the second drone
        ################################################################
        commander.go_to(x_04, y_04, altitude, 0, flight_time, relative=False)
        time.sleep(sleep_time)

       
        ################################################################
        # Second drone performs a square sequence
        ################################################################
        commander.go_to(x_04, y_04 - square_edge, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_04 + square_edge, y_04 - square_edge, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_04 + square_edge, y_04, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

        commander.go_to(x_04, y_04, altitude, 0, flight_time, relative=False)
        time.sleep(flight_time)

    time.sleep(2)  # Wait for the drones to settle down after the sequence


uris = {
    'radio://0/80/2M/E7E7E7E701',

    'radio://0/80/2M/E7E7E7E703',

    'radio://1/80/2M/E7E7E7E704',

    'radio://1/80/2M/E7E7E7E706',

    # Add more URIs if you want more copters in the swarm
    # URIs in a swarm using the same radio must also be on the same channel
}

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        print('Connected to Crazyflies')

        #### Reset Kalman estimator for all drones. Though not necessary for absolute positioning. Only reset for relative movement. ####
        print('Resetting estimators...')
        swarm.reset_estimators()
        print('Done resetting estimators')
        time.sleep(3)   

        swarm.parallel_safe(take_off)
        time.sleep(2)
        swarm.parallel_safe(square_motion)
        swarm.parallel_safe(land)