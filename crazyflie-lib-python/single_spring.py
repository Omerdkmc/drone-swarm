import time
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crtp import init_drivers
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander

uri='radio://0/60/2M/E7E7E7E708'
DT = 0.1  # saniye
HOVER_HEIGHT = 1.5
DURATION = 30
k = 0.1
b=0.035 # damping coefficient
mass = 0.035
spring_length = 1.5
center=(2.05, 2.4)
fixed_points= [(center[0]-spring_length,center[1]),(center[0],center[1]-spring_length),(center[0],center[1]+spring_length),(center[0]+spring_length,center[1])]

def spring_force(position, fixed_points, spring_length, k, velocity):
    force = np.zeros(2)
    for fixed_pos in fixed_points:
        delta = np.array(fixed_pos) - np.array(position)
        distance = np.linalg.norm(delta)
        if distance > 1e-6:
            direction = delta / distance
            force += k * (distance - spring_length) * direction 
    force -= b * velocity  # damping force
    return force

def get_position(cf):
    log_conf = LogConfig(name='kalman_sync', period_in_ms=10)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    with SyncLogger(cf, log_conf) as logger:
        for entry in logger:
            data = entry[1]
            x = data['kalman.stateX']
            y = data['kalman.stateY']
            return np.array([x, y])


def reset_estimator(cf):
    cf.param.set_value('kalman.robustTdoa', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.1)
    cf.param.set_value('kalman.robustTdoa', '1')
    time.sleep(0.1)

def takeoff(cf, height=HOVER_HEIGHT, duration=4.0):
    cf.high_level_commander.takeoff(height, duration)
    time.sleep(1.5)
    cf.high_level_commander.go_to(center[0]+0.5, center[1], height, 0, 3.0)
    time.sleep(duration + duration + 5.0)

def land(cf):
    cf.high_level_commander.go_to(center[0], center[1], 1, 0, 2.0)
    cf.high_level_commander.go_to(center[0], center[1], 0.5, 0, 2.0)
    cf.high_level_commander.land(0.1, 3.0)
    time.sleep(3.5)
    cf.high_level_commander.stop()

def run_single_spring():
    init_drivers()
    cf = Crazyflie(rw_cache='./cache')

    with SyncCrazyflie(uri, cf=cf) as scf:
        print("🔄 Resetting estimator...")
        reset_estimator(cf)

        print("🛫 Takeoff...")
        takeoff(cf)
        print("🔁 Starting spring simulation...")
        velocity = np.zeros(2)
        t0 = time.time()

        while time.time() - t0 < DURATION:
            position = get_position(cf)
            force = spring_force(position, fixed_points, spring_length, k, velocity)
            acc = force / mass
            velocity += acc * DT
            cf.commander.send_hover_setpoint(velocity[0],velocity[1], 0, HOVER_HEIGHT)
            print(f"Position: {position}, Velocity: {velocity}, Force: {force}")
            time.sleep(DT)

        print("🛬 Landing...")
        # cf.high_level_commander.land(0.1, 4.0)
        land(cf)
        time.sleep(5.0)
        cf.high_level_commander.stop()

if __name__ == '__main__':
    run_single_spring()
