import time
import numpy as np
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie import Crazyflie
from cflib.crtp import init_drivers
from cflib.crazyflie.swarm import CachedCfFactory


from cflib.crazyflie.log import LogConfig
import os

# ---- CONFIG ----
URIS = [
    'radio://0/50/2M/E7E7E7E701',
    'radio://0/50/2M/E7E7E7E708',
    'radio://0/50/2M/E7E7E7E703',
    'radio://1/60/2M/E7E7E7E704',
    'radio://1/60/2M/E7E7E7E705',
    'radio://1/60/2M/E7E7E7E706',
    'radio://2/70/2M/E7E7E7E707',
]

pid_dict = {
    'radio://0/80/2M/E7E7E7E701': (2.02, 0.025, 0.7),
    'radio://0/80/2M/E7E7E7E703': (2.02, 0.025, 0.7),
    'radio://1/80/2M/E7E7E7E704': (2.02, 0.025, 0.7),
    'radio://1/80/2M/E7E7E7E706': (2.02, 0.025, 0.7),
    'radio://1/80/2M/E7E7E7E701': (2.02, 0.025, 0.7),
    'radio://0/80/2M/E7E7E7E703': (2.02, 0.025, 0.7),
    'radio://1/80/2M/E7E7E7E704': (2.02, 0.025, 0.7),
    'radio://1/80/2M/E7E7E7E706': (2.02, 0.025, 0.7),
}

zKp, zKi, zKd = 2.02, 0.025, 0.7  # PID parameters for Z-axis
xKp, xKi, xKd = 2.0, 0.5, 0.0  # PID parameters for X-axis
yKp, yKi, yKd = 2.0, 0.5, 0.0  # PID parameters for Y-axis

HOVER_HEIGHT = 1.25  # meters
DURATION = 20       # seconds to hover

# ---- HEXAGON POSITIONS ----
radius = 1.75
hex_xy = [(radius * np.cos(a)+2.05, radius * np.sin(a)+2.4) for a in np.linspace(0, 2*np.pi, 6, endpoint=False)]
hex_xy.append((2.05, 2.4))  # center drone
POSITIONS = [(x, y, HOVER_HEIGHT) for (x, y) in hex_xy]  # (x,y,z) per drone


filename = 'C:/Users/omerr/drone_win/data_xyz/hexagon_xyz.csv'
figurename = 'hexagon_xyz.jpg'
gifname = 'hexagon_xyz.gif'

def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']

    with open(filename, 'a') as f:
        f.write('{},{},{},{}\n'.format(timestamp, x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

# ---- FLIGHT COMMANDS ----
def take_off(cf, position, height=HOVER_HEIGHT, duration=3.0):
    cf.high_level_commander.takeoff(height, duration)
    time.sleep(duration + 1.0)

def go_to(cf, position, duration=3.0):
    x, y, z = position
    cf.high_level_commander.go_to(x, y, z, 0, duration, relative=False)
    time.sleep(duration + 0.5)

def hover(cf, duration=DURATION):
    time.sleep(duration)

def land(cf, duration=5.0):
    cf.high_level_commander.land(0.1, duration)
    time.sleep(duration + 1.0)

def set_pid(scf, pid_vals):
    zKp, zKi, zKd = pid_vals
    scf.cf.param.set_value('posCtlPid.zKp', zKp)
    scf.cf.param.set_value('posCtlPid.zKi', zKi)
    scf.cf.param.set_value('posCtlPid.zKd', zKd)

# ---- CALLBACK ----
def run_sequence(scf, uri):
    cf = scf.cf
    index = URIS.index(uri)
    pos = POSITIONS[index]

    # zKp, zKi, zKd = 2.02, 0.025, 0.7  # Default PID values
    # scf.cf.param.set_value('posCtlPid.zKp', zKp)
    # scf.cf.param.set_value('posCtlPid.zKi', zKi)
    # scf.cf.param.set_value('posCtlPid.zKd', zKd)
    
    take_off(cf, pos)
    go_to(cf, pos)
    hover(cf)
    land(cf)

# ---- MAIN ----
if __name__ == '__main__':
    init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(URIS, factory=factory) as swarm:
    

        print('Resetting estimators...')
        swarm.reset_estimators()
        print('Estimators reset, arming motors!')
        # Pass both scf and uri to run_sequence
        args_dict = {uri: (uri,) for uri in URIS}
        # swarm.start_position_printing
        swarm.parallel_safe(run_sequence, args_dict=args_dict)
