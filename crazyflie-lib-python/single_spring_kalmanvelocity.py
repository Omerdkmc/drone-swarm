import time
import numpy as np
import csv
from cflib.crazyflie import Crazyflie
from cflib.crtp import init_drivers
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
import matplotlib.pyplot as plt
import pandas as pd
import imageio

uri='radio://0/60/2M/E7E7E7E707'
DT = 0.1  # saniye
HOVER_HEIGHT = 1.5
DURATION = 20
k = 0.1
b=0.0325 # damping coefficient
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
        

def get_position_and_velocity(cf):
    log_conf = LogConfig(name='kalman_sync', period_in_ms=10)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.statePX', 'float')
    log_conf.add_variable('kalman.statePY', 'float')
    with SyncLogger(cf, log_conf) as logger:
        for entry in logger:
            data = entry[1]
            x = data['kalman.stateX']
            y = data['kalman.stateY']
            vx = data['kalman.statePX']
            vy = data['kalman.statePY']
            return np.array([x, y]), np.array([vx, vy])
        

def reset_estimator(cf):
    cf.param.set_value('kalman.robustTdoa', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.1)
    cf.param.set_value('kalman.robustTdoa', '1')
    time.sleep(0.1)


def smooth_landing_high_level(scf):
    hlc = scf.cf.high_level_commander
    # Move to center at current height
    hlc.go_to(center[0], center[1], HOVER_HEIGHT, 0, 2.0, relative=False)
    time.sleep(2.0)
    # Descend in steps
    altitudes = [1.0, 0.7, 0.4, 0.2]
    for alt in altitudes:
        print(f"Descending to {alt} m")
        hlc.go_to(center[0], center[1], alt, 0, 2.0, relative=False)
        time.sleep(2.0)
    # Final land command
    print("Final landing...")
    hlc.land(0.0, 2.0)
    time.sleep(2.5)
    hlc.stop()
    time.sleep(1.0)

def takeoff(cf, height=HOVER_HEIGHT, duration=4.0):
    # cf.high_level_commander.stop()
    # time.sleep(1.0)
    cf.high_level_commander.takeoff(height, duration)
    time.sleep(1.5)
    cf.high_level_commander.go_to(center[0], center[1], height, 0, 3.0)
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
        time.sleep(1.0)
        reset_estimator(cf)
        time.sleep(1.0)
        print("🛫 Taking off...")
        takeoff(cf)
        time.sleep(1.0)
        print('Moving to start position...')
        cf.high_level_commander.go_to(center[0]+1.5, center[1]+1.5, HOVER_HEIGHT, 0, 3.0)

        time.sleep(1.0)
        print("🔁 Starting spring simulation...")
        time.sleep(1.0)

        t0 = time.time()
        data_log = []  # To store [time, pos_x, pos_y, force_x, force_y]

        while time.time() - t0 < DURATION:
            now = time.time() - t0
            position, velocity = get_position_and_velocity(cf)
            force = spring_force(position, fixed_points, spring_length, k, velocity)
            acc = force / mass
            velocity += acc * DT
            cf.commander.send_hover_setpoint(velocity[0], velocity[1], 0, HOVER_HEIGHT)
            data_log.append([now, position[0], position[1], force[0], force[1]])
            time.sleep(DT)

        # Stabilize at center before landing
        cf.commander.send_notify_setpoint_stop()
        cf.high_level_commander.go_to(center[0], center[1], HOVER_HEIGHT, 0, 2.0)
        time.sleep(2.0)

        print("🛬 Landing...")
        smooth_landing_high_level(scf)
        time.sleep(1.0)

        # Save to CSV
        with open('C:/Users/omerr/drone-swarm/data_spring/data_spring_data_kalmanvelocity.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'pos_x', 'pos_y', 'force_x', 'force_y'])
            writer.writerows(data_log)
        print("Data saved to C:/Users/omerr/drone-swarm/data_spring_data_kalmanvelocity.csv")



if __name__ == '__main__':
    run_single_spring()

    # # Plotting after data collection
    # csv_path = 'C:/Users/omerr/drone-swarm/data_spring/data_spring_data_kalmanvelocity.csv'
    # df = pd.read_csv(csv_path)

    # fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)

    # # First row: position vs time
    # axes[0, 0].plot(df['time'], df['pos_x'], label='X Position')
    # axes[0, 0].set_ylabel('X Position (m)')
    # axes[0, 0].set_title('X Position vs Time')
    # axes[0, 0].grid()

    # axes[0, 1].plot(df['time'], df['pos_y'], label='Y Position', color='orange')
    # axes[0, 1].set_ylabel('Y Position (m)')
    # axes[0, 1].set_title('Y Position vs Time')
    # axes[0, 1].grid()

    # # Second row: force vs time
    # axes[1, 0].plot(df['time'], df['force_x'], label='X Force', color='green')
    # axes[1, 0].set_ylabel('X Force (N)')
    # axes[1, 0].set_xlabel('Time (s)')
    # axes[1, 0].set_title('X Force vs Time')
    # axes[1, 0].grid()

    # axes[1, 1].plot(df['time'], df['force_y'], label='Y Force', color='red')
    # axes[1, 1].set_ylabel('Y Force (N)')
    # axes[1, 1].set_xlabel('Time (s)')
    # axes[1, 1].set_title('Y Force vs Time')
    # axes[1, 1].grid()

    # plt.tight_layout()
    # plt.savefig('C:/Users/omerr/drone-swarm/data_spring/data_spring_data_kalmanvelocity_plot.png', dpi=300)
    # plt.show()

    # # Create trajectory GIF
    # frames = []
    # N = len(df)
    # fade_steps = 30  # Number of points to fade

    # for i in range(fade_steps, N):
    #     fig, ax = plt.subplots(figsize=(6, 6))
    #     # Plot faded path
    #     for j in range(fade_steps):
    #         idx = i - fade_steps + j
    #         alpha = j / fade_steps
    #         ax.plot(df['pos_x'][idx:idx+2], df['pos_y'][idx:idx+2], color='blue', alpha=alpha, linewidth=2)
    #     # Plot current position
    #     ax.scatter(df['pos_x'][i], df['pos_y'][i], color='red', s=80, label='Current')
    #     ax.set_xlim(df['pos_x'].min()-0.5, df['pos_x'].max()+0.5)
    #     ax.set_ylim(df['pos_y'].min()-0.5, df['pos_y'].max()+0.5)
    #     ax.set_xlabel('X Position (m)')
    #     ax.set_ylabel('Y Position (m)')
    #     ax.set_title('Drone Trajectory')
    #     ax.legend()
    #     plt.tight_layout()
    #     # Save frame to buffer
    #     fig.canvas.draw()
    #     image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
    #     image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    #     frames.append(image)
    #     plt.close(fig)

    # # Save as GIF
    # gif_path = 'C:/Users/omerr/drone-swarm/data_spring/trajectory.gif'
    # imageio.mimsave(gif_path, frames, duration=0.05)
    # print(f"Trajectory GIF saved to {gif_path}")
