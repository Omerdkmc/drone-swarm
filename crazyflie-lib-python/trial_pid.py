import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

URI = 'radio://0/80/2M/E7E7E7E705'  # Change to your drone's URI
filename = 'position_log_pidxy_tuned_deneme.csv'


def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(1.5, 3.0)
    time.sleep(5.0)

def land(scf):
    commander = scf.cf.high_level_commander

    commander.land(0.20, 4.0)
    time.sleep(2)

    commander.stop()

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(2.0)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2.0)


def position_callback(timestamp, data, logconf):
    z = data['kalman.stateZ']
    x = data['kalman.stateX']
    y = data['kalman.stateY']

    print('pos: ({},{},{})'.format(x,y,z))

    with open(filename, 'a') as f:
        f.write('{},{},{},{}\n'.format(timestamp, x, y, z))

def run_sequence_setpoint(scf, sequence):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))
        for i in range(100):
            cf.commander.send_position_setpoint(1.0,
                                                1.0,
                                                0.5,
                                                0.0)
            time.sleep(0.1)


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()
    return log_conf  # Return the log configuration for later use


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Example: Set pitch PID value
        hover_time = 20
        scf.cf.param.set_value('posCtlPid.zKp', 1.0)
        scf.cf.param.set_value('posCtlPid.zKi', 0.0)
        scf.cf.param.set_value('posCtlPid.zKd', 0.0)

        scf.cf.param.set_value('posCtlPid.xKp', 1.0)
        scf.cf.param.set_value('posCtlPid.xKi', 0.0)
        scf.cf.param.set_value('posCtlPid.xKd', 0.0)

        scf.cf.param.set_value('posCtlPid.yKp', 1.0)
        scf.cf.param.set_value('posCtlPid.yKi', 0.0)
        scf.cf.param.set_value('posCtlPid.yKd', 0.0)
        # Repeat for roll and yaw as needed
        print("PID parameters updated.")
        time.sleep(1)

        print("Resetting estimators...")
        reset_estimator(scf)

        print("Taking off...")
        take_off(scf)
        time.sleep(2.5)
        log_conf = start_position_printing(scf)
        print(f'Hovering for {hover_time} seconds...')
        time.sleep(hover_time)
        log_conf.stop()
        land(scf)


        print("Plotting and saving data...")
        df = pd.read_csv(filename, header=None, names=['timestamp','x', 'y', 'z'])

        df['time_s'] = (df['timestamp'] - df['timestamp'].iloc[0]) / 1000.0  # adjust divisor if needed

        plt.figure()
        plt.plot(df['time_s'], df['x'], label='X Position')

        mean_x = df['x'].mean()
        var_x = df['x'].var()

        mean_y = df['y'].mean()
        var_y = df['y'].var()

        # Annotate mean and variance on the plot
        plt.text(df['time_s'].iloc[len(df)//2], mean_x + 0.01,
                 f'Mean x: {mean_x:.3f} m\nVariance x: {var_x:.5f}',
                 fontsize=12, color='darkred', ha='center', va='bottom')

        plt.xlabel('Time (s)')
        plt.ylabel('X Position (m)')
        plt.title('Drone x Over Time')
        #plt.axhline(y=1.5, color='red', linestyle='--', linewidth=2, label='Setpoint = 1.5m')

        plt.grid()
        plt.legend()
        plt.tight_layout()
        plt.savefig('C:/Users/omerr/drone_win/data/position_log_pidx3_jpg', dpi=300, bbox_inches='tight')
        plt.show()



        plt.figure()
        plt.plot(df['time_s'], df['y'], label='Y Position')

        mean_x = df['x'].mean()
        var_x = df['x'].var()

        mean_y = df['y'].mean()
        var_y = df['y'].var()

        # Annotate mean and variance on the plot
        plt.text(df['time_s'].iloc[len(df)//2], mean_x + 0.01,
                 f'Mean y: {mean_y:.3f} m\nVariance y: {var_y:.5f}',
                 fontsize=12, color='darkred', ha='center', va='top')

        plt.xlabel('Time (s)')
        plt.ylabel('Y Position (m)')
        plt.title('Drone y Over Time')
        #plt.axhline(y=1.5, color='red', linestyle='--', linewidth=2, label='Setpoint = 1.5m')
        
        plt.grid()
        plt.legend()
        plt.tight_layout()
        plt.savefig('C:/Users/omerr/drone_win/data/position_log_pidy3_jpg', dpi=300, bbox_inches='tight')
        plt.show()
