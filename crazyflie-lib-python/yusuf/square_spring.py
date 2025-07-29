import os
import time
import numpy as np
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URIs for our four Crazyflies
URIS = [
    'radio://0/20/2M/E7E7E7E705',
    'radio://1/60/2M/E7E7E7E706',
    'radio://1/60/2M/E7E7E7E707',
    'radio://1/60/2M/E7E7E7E708',
]

# Simulation parameters
DT = 0.05           # timestep (s)
DURATION = 30       # total sim time (s)
HOVER_HEIGHT = 1.75 # fixed altitude (m)
k = 0.05             # spring constant
b = 0.035           # damping coefficient
mass = 0.035        # drone mass (kg)
l = 1.5             # rest length of springs (m)
center = (2.05, 2.4)

# Initial square corner positions (clockwise from bottom-left)
initial_positions = [
    (center[0] - l/2, center[1] - l/2),
    (center[0] + l/2, center[1] - l/2),
    (center[0] + l/2, center[1] + l/2),
    (center[0] - l/2, center[1] + l/2),
]

# Define which drones are connected by springs (edges of square + diagonals)
connections = {
    0: [1, 3],
    1: [0, 2],
    2: [1, 3],
    3: [0, 2],
}
# diagonal connections
connections2 = {
    0: [2],
    1: [3],
    2: [0],
    3: [1],
}

def spring_force(positions, idx, connections, connections2, rest, k, b, velocity):
    """Compute net spring + damping force on drone idx."""
    p0 = np.array(positions[idx])
    F = np.zeros(2)
    # square edges
    for j in connections[idx]:
        pj = np.array(positions[j])
        delta = pj - p0
        d = np.linalg.norm(delta)
        if d > 1e-6:
            F += k * (d - rest) * (delta / d)
    # diagonal
    for j in connections2[idx]:
        pj = np.array(positions[j])
        delta = pj - p0
        d = np.linalg.norm(delta)
        if d > 1e-6:
            F += k * (d - rest * np.sqrt(2)) * (delta / d)
    # damping
    F -= b * velocity
    return F

def get_position_and_velocity(cf):
    """Read Kalman position & velocity once."""
    lc = LogConfig(name='kal', period_in_ms=int(DT*1000))
    lc.add_variable('kalman.stateX', 'float')
    lc.add_variable('kalman.stateY', 'float')
    lc.add_variable('kalman.statePX', 'float')
    lc.add_variable('kalman.statePY', 'float')
    with SyncLogger(cf, lc) as logger:
        for entry in logger:
            d = entry[1]
            return np.array([d['kalman.stateX'], d['kalman.stateY']]), \
                   np.array([d['kalman.statePX'], d['kalman.statePY']])

# --- Per-drone helper functions for Swarm ---

def take_off(scf: SyncCrazyflie):
    scf.cf.high_level_commander.takeoff(HOVER_HEIGHT, 3.0)

def go_to_initial_position_high_level(scf: SyncCrazyflie, pos: tuple):
    x, y = pos
    scf.cf.high_level_commander.go_to(x, y, HOVER_HEIGHT, 0, 3.0)

def stop_hover_setpoint(scf: SyncCrazyflie):
    # stop any low-level hover setpoints
    scf.cf.commander.send_notify_setpoint_stop()

def smooth_landing_high_level(scf: SyncCrazyflie):
    hlc = scf.cf.high_level_commander
    # descent steps
    for alt in [1.0, 0.7, 0.4, 0.2]:
        hlc.go_to(center[0], center[1], alt, 0, 2.0)
        time.sleep(2.0)
    hlc.land(0.0, 2.0)
    time.sleep(2.5)
    hlc.stop()

# --- Collective spring simulation step ---

def spring_simulation_step(t: float, scf_map: dict):
    """
    Called in lock-step every DT seconds.
    scf_map maps uri -> SyncCrazyflie instance.
    """
    # 1) read all positions & velocities
    pos = {}
    vel = {}
    for uri, scf in scf_map.items():
        p, v = get_position_and_velocity(scf.cf)
        pos[uri] = p
        vel[uri] = v
    # preserve ordering
    uris = list(scf_map.keys())
    positions = [pos[u] for u in uris]
    # 2) compute new velocities
    new_vel = {}
    for i, uri in enumerate(uris):
        F = spring_force(
            positions,
            i,
            connections,
            connections2,
            rest=l,
            k=k,
            b=b,
            velocity=vel[uri]
        )
        new_vel[uri] = vel[uri] + (F/mass) * DT
    # 3) send hover setpoints
    for uri, scf in scf_map.items():
        vx, vy = new_vel[uri]
        scf.cf.commander.send_hover_setpoint(vx, vy, 0, HOVER_HEIGHT)

# --- Main script using Swarm ---

if __name__ == '__main__':
    try:
        init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')

        # clean up any old logs
        for uri in URIS:
            fn = f"data_{uri[-2:]}.csv"
            if os.path.exists(fn):
                os.remove(fn)
        time.sleep(1.0)

        with Swarm(URIS, factory=factory) as swarm:
            print("🔄 Resetting estimators...")
            swarm.reset_estimators()
            time.sleep(1.5)

            print("🛫 Taking off all drones...")
            swarm.parallel_safe(take_off)
            time.sleep(5.0)

            print("🚀 Moving to initial square corners...")
            init_args = {uri: (initial_positions[i],) for i, uri in enumerate(URIS)}
            swarm.parallel_safe(go_to_initial_position_high_level, args_dict=init_args)
            time.sleep(3.0)

            print("🔁 Starting 4-drone spring simulation...")
            swarm.parallel(spring_simulation_step, duration=DURATION, dt=DT)
            print("✅ Spring simulation complete.")

            print("✋ Stopping hover setpoints before landing...")
            swarm.parallel_safe(stop_hover_setpoint)
            time.sleep(1.0)

            print("↪️  Returning all drones to center...")
            center_args = {uri: (center, ) for uri in URIS}
            swarm.parallel_safe(go_to_initial_position_high_level, args_dict=center_args)
            time.sleep(2.0)

            print("🛬 Landing all drones...")
            swarm.parallel_safe(smooth_landing_high_level)

            print("🎉 All done, logged data to CSV files per drone.")
    except(Exception, KeyboardInterrupt) as e:
        swarm.parallel_safe(smooth_landing_high_level)
