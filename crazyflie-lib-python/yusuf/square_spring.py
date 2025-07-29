import os
import time
import numpy as np
from cflib.crtp import init_drivers
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URIs for the four Crazyflies
URIS = [
    'radio://0/20/2M/E7E7E7E705',
    'radio://0/20/2M/E7E7E7E706',
    'radio://1/60/2M/E7E7E7E707',
    'radio://1/60/2M/E7E7E7E708',
]

# Simulation parameters
DT = 0.2           # timestep (s)
DURATION = 30      # total simulation time (s)
HOVER_HEIGHT = 1.25  # fixed altitude (m)
k = 0.07           # spring constant
b = 0.03          # damping coefficient
mass = 0.035       # effective mass (kg)
l = 1.5            # rest length of springs (m)
center = (2.05, 2.4)

# Which drones are connected in the square + diagonals
connections = {0: [1, 3], 1: [0, 2], 2: [1, 3], 3: [0, 2]}
diag       = {0: [2],    1: [3],    2: [0],    3: [1]}

def get_position_and_velocity(cf):
    """Grab one Kalman sample of X,Y and PX,PY."""
    lc = LogConfig(name='kv', period_in_ms=int(DT*1000))
    lc.add_variable('kalman.stateX', 'float')
    lc.add_variable('kalman.stateY', 'float')
    lc.add_variable('kalman.statePX', 'float')
    lc.add_variable('kalman.statePY', 'float')
    with SyncLogger(cf, lc) as logger:
        for entry in logger:
            d = entry[1]
            return (
                np.array([d['kalman.stateX'],  d['kalman.stateY']]),
                np.array([d['kalman.statePX'], d['kalman.statePY']])
            )

def spring_force(positions, idx):
    """Compute net spring + diagonal force on drone idx."""
    p0 = positions[idx]
    F  = np.zeros(2)
    # square edges
    for j in connections[idx]:
        dvec = positions[j] - p0
        dist = np.linalg.norm(dvec)
        if dist > 1e-6:
            F += k * (dist - l) * (dvec / dist)
    # diagonal
    for j in diag[idx]:
        dvec = positions[j] - p0
        dist = np.linalg.norm(dvec)
        if dist > 1e-6:
            F += k * (dist - l*np.sqrt(2)) * (dvec / dist)
    return F

def spring_worker(scf: SyncCrazyflie, swarm: Swarm):
    """
    Runs on each drone thread via parallel_safe.
    Loops for DURATION, reading all positions, computing its own spring+damping
    force, updating velocity, and sending hover setpoints.
    """
    uri = scf.cf.link_uri
    idx = URIS.index(uri)
    t0  = time.time()

    # get initial velocity
    _, v = get_position_and_velocity(scf.cf)

    while time.time() - t0 < DURATION:
        # 1) gather everyone's state
        positions = []
        velocities = []
        for other in URIS:
            other_scf = swarm._cfs[other]  # SyncCrazyflie for that URI
            p, vv     = get_position_and_velocity(other_scf.cf)
            positions.append(p)
            velocities.append(vv)
        positions  = np.array(positions)
        velocities = np.array(velocities)

        # 2) compute force + update velocity
        F = spring_force(positions, idx)
        F -= b * velocities[idx]
        v  = velocities[idx] + (F / mass) * DT

        # 3) send it to the drone
        scf.cf.commander.send_hover_setpoint(v[0], v[1], 0, HOVER_HEIGHT)
        time.sleep(DT)

if __name__ == '__main__':
    init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    # clean up any old data files
    for uri in URIS:
        fn = f"data_{uri[-2:]}.csv"
        if os.path.exists(fn):
            os.remove(fn)
    time.sleep(0.5)

    with Swarm(URIS, factory=factory) as swarm:
        # 1) reset and take off
        print("🔄 Resetting estimators...")
        swarm.reset_estimators()
        time.sleep(1.0)

        print("🛫 Taking off all drones...")
        def takeoff(scf): scf.cf.high_level_commander.takeoff(HOVER_HEIGHT, 3.0)
        swarm.parallel_safe(takeoff)
        time.sleep(5.0)

        # 2) move into initial square
        print("🚀 Move to square corners...")
        corners = [
            (center[0]-l/2, center[1]-l/2),
            (center[0]+l/2, center[1]-l/2),
            (center[0]+l/2, center[1]+l/2),
            (center[0]-l/2, center[1]+l/2),
        ]
        def go_to_corner(scf, pos):
            x,y = pos
            scf.cf.high_level_commander.go_to(x, y, HOVER_HEIGHT, 0, 3.0)
        args = {uri:(corners[i],) for i,uri in enumerate(URIS)}
        swarm.parallel_safe(go_to_corner, args_dict=args)
        time.sleep(5.0)

        # 3) spring simulation
        print("🔁 Running 4-drone spring sim...")
        swarm.parallel_safe(spring_worker, args_dict={uri:(swarm,) for uri in URIS})
        print("✅ Simulation complete.")

        # 4) stop any leftover hover setpoints
        def stop_hover(scf):
            scf.cf.commander.send_notify_setpoint_stop()
        swarm.parallel_safe(stop_hover)

        # 5) return to center & land
        print("↪️ Returning to center & landing...")
        def land(scf):
            hlc = scf.cf.high_level_commander
            hlc.go_to(center[0], center[1], 0.5, 0, 2.0)
            hlc.land(0.0, 2.0)
            time.sleep(2.5)
        swarm.parallel_safe(land)

        print("🎉 All done.")