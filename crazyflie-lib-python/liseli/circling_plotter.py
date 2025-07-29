import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import glob

# List of log files (update path if needed)
log_files = glob.glob("C:/Users/omerr/drone-swarm/crazyflie-lib-python/liseli/data_circling_square_*.csv")
num_drones = len(log_files)
colors = plt.cm.get_cmap('tab10', num_drones)  # Use tab10 colormap for up to 10 drones

# Read all drone data
drone_data = []
for log_file in log_files:
    df = pd.read_csv(log_file, header=None, names=['timestamp', 'x', 'y', 'z'])
    df['time_s'] = (df['timestamp'] - df['timestamp'].iloc[0]) / 1000.0
    drone_data.append(df)

# Find the max length for animation
max_len = max([len(df) for df in drone_data])

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, 4.5)
ax.set_ylim(0, 4.5)
ax.set_title("Drone Trajectories with Fading Traces")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")


# Number of points for gradient fade
fade_len = 20
# Create lists to hold gradient lines for each drone
gradient_lines = []
for i in range(num_drones):
    drone_lines = [ax.plot([], [], color=colors(i), lw=2, alpha=0)[0] for _ in range(fade_len)]
    gradient_lines.append(drone_lines)
# Faded full trace (background)
fading_traces = [ax.plot([], [], color=colors(i), alpha=0.15, lw=1)[0] for i in range(num_drones)]

def animate(frame):
    artists = []
    for i, df in enumerate(drone_data):
        if frame < len(df):
            # Gradient fade for last fade_len points
            for j in range(fade_len):
                idx = frame - fade_len + j
                if idx < 1 or idx >= len(df):
                    gradient_lines[i][j].set_data([], [])
                    gradient_lines[i][j].set_alpha(0)
                else:
                    x_vals = df['x'].iloc[idx-1:idx+1]
                    y_vals = df['y'].iloc[idx-1:idx+1]
                    alpha = (j+1)/fade_len  # 0..1 gradient
                    gradient_lines[i][j].set_data(x_vals, y_vals)
                    gradient_lines[i][j].set_alpha(alpha)
                artists.append(gradient_lines[i][j])
            # Full trace (faded background)
            fading_traces[i].set_data(df['x'].iloc[:frame], df['y'].iloc[:frame])
            artists.append(fading_traces[i])
    return artists

ani = animation.FuncAnimation(fig, animate, frames=max_len, interval=50, blit=True)

ani.save("C:/Users/omerr/drone-swarm/crazyflie-lib-python/liseli/drone_circling_square.gif", writer='pillow', fps=20)
plt.close(fig)
print("Animated GIF saved as drone_circling_square.gif")