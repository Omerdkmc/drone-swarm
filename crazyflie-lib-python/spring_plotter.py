# Plotting after data collection
import matplotlib.pyplot as plt
import pandas as pd
import imageio
import numpy as np

def plotter():
    csv_path = 'C:/Users/omerr/drone-swarm/data_spring/data_spring_data_kalmanvelocity.csv'
    df = pd.read_csv(csv_path)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)

    # First row: position vs time
    axes[0, 0].plot(df['time'], df['pos_x'], label='X Position')
    axes[0, 0].set_ylabel('X Position (m)')
    axes[0, 0].set_title('X Position vs Time')
    axes[0, 0].grid()

    axes[0, 1].plot(df['time'], df['pos_y'], label='Y Position', color='orange')
    axes[0, 1].set_ylabel('Y Position (m)')
    axes[0, 1].set_title('Y Position vs Time')
    axes[0, 1].grid()

    # Second row: force vs time
    axes[1, 0].plot(df['time'], df['force_x'], label='X Force', color='green')
    axes[1, 0].set_ylabel('X Force (N)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_title('X Force vs Time')
    axes[1, 0].grid()

    axes[1, 1].plot(df['time'], df['force_y'], label='Y Force', color='red')
    axes[1, 1].set_ylabel('Y Force (N)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_title('Y Force vs Time')
    axes[1, 1].grid()

    plt.tight_layout()
    plt.savefig('C:/Users/omerr/drone-swarm/data_spring/data_spring_data_kalmanvelocity_plot.png', dpi=300)
    plt.show()

    # Create trajectory GIF
    frames = []
    N = len(df)
    fade_steps = 30  # Number of points to fade

    for i in range(fade_steps, N):
        fig, ax = plt.subplots(figsize=(6, 6))
        # Plot faded path
        for j in range(fade_steps):
            idx = i - fade_steps + j
            alpha = j / fade_steps
            ax.plot(df['pos_x'][idx:idx+2], df['pos_y'][idx:idx+2], color='blue', alpha=alpha, linewidth=2)
        # Plot current position
        ax.scatter(df['pos_x'][i], df['pos_y'][i], color='red', s=80, label='Current')
        ax.set_xlim(df['pos_x'].min()-0.5, df['pos_x'].max()+0.5)
        ax.set_ylim(df['pos_y'].min()-0.5, df['pos_y'].max()+0.5)
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Drone Trajectory')
        ax.legend()
        # Add time annotation
        ax.text(0.05, 0.95, f"Time: {df['time'][i]:.2f} s", transform=ax.transAxes,
                fontsize=14, color='black', verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
        plt.tight_layout()
        # Save frame to buffer
        fig.canvas.draw()
        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
        image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        frames.append(image)
        plt.close(fig)

    # Save as GIF
    gif_path = 'C:/Users/omerr/drone-swarm/data_spring/trajectory.gif'
    imageio.mimsave(gif_path, frames, duration=0.05)
    print(f"Trajectory GIF saved to {gif_path}")

if __name__ == "__main__":
    plotter()