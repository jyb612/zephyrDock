import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
from matplotlib.collections import LineCollection
from datetime import datetime, timedelta

# Load the data
log_path = os.path.join(os.path.expanduser('~'), 'zephyrDock', 'zephyrDock', 'logs', 'odometry_20250403_024114.csv')
df = pd.read_csv(log_path)

# 1. Reverse Z values (convert FRD to NED for visualization)
df['z_display'] = -df['z']  # Flip the sign for visualization

# 2. Calculate actual duration
start_time = datetime.fromtimestamp(df['timestamp'].iloc[0])
end_time = datetime.fromtimestamp(df['timestamp'].iloc[-1])
duration = end_time - start_time
duration_str = str(timedelta(seconds=duration.total_seconds())).split(".")[0]  # Remove microseconds

# Create normalized time values (0-1) for coloring
normalized_time = np.linspace(0, 1, len(df))

# Convert to numpy arrays
x = df['x'].values
y = df['y'].values
z_display = df['z_display'].values
timestamps = df['timestamp'].values

# Create figure with larger size
plt.figure(figsize=(14, 6))

# --- 2D XY Plot with Color Gradient ---
ax1 = plt.subplot(131)
points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

lc = LineCollection(segments, cmap='viridis', norm=plt.Normalize(0, 1))
lc.set_array(normalized_time)
lc.set_linewidth(2)
line = ax1.add_collection(lc)

# Add start/end markers
ax1.scatter(x[0], y[0], c='green', s=100, label=f'Start\n{datetime.fromtimestamp(timestamps[0]).strftime("%H:%M:%S")}', zorder=3)
ax1.scatter(x[-1], y[-1], c='red', s=100, label=f'End\n{datetime.fromtimestamp(timestamps[-1]).strftime("%H:%M:%S")}', zorder=3)

ax1.set_xlabel('X Position (m)')
ax1.set_ylabel('Y Position (m)')
ax1.set_title(f'XY Trajectory\nTotal Duration: {duration_str}')
ax1.grid(True)
ax1.legend()
plt.colorbar(line, ax=ax1, label='Progress')

# --- 3D Plot with Color Gradient ---
ax2 = plt.subplot(132, projection='3d')
for i in range(len(x)-1):
    ax2.plot(x[i:i+2], y[i:i+2], z_display[i:i+2], 
             color=plt.cm.viridis(normalized_time[i]),
             alpha=0.8, linewidth=2)

# Add start/end markers
ax2.scatter(x[0], y[0], z_display[0], c='green', s=100, label='Start')
ax2.scatter(x[-1], y[-1], z_display[-1], c='red', s=100, label='End')

ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_zlabel('Altitude (m)')  # Changed from Z to Altitude
ax2.set_title('3D Flight Path')

# --- Altitude Profile with Correct Time ---
ax3 = plt.subplot(133)
relative_time = timestamps - timestamps[0]  # Seconds since start

for i in range(len(z_display)-1):
    ax3.plot([relative_time[i], relative_time[i+1]], 
             [z_display[i], z_display[i+1]], 
             color=plt.cm.viridis(normalized_time[i]),
             linewidth=2)

ax3.scatter(relative_time[0], z_display[0], c='green', s=100, label='Start')
ax3.scatter(relative_time[-1], z_display[-1], c='red', s=100, label='End')

ax3.set_xlabel(f'Time (seconds)\nStart: {datetime.fromtimestamp(timestamps[0]).strftime("%H:%M:%S")}')
ax3.set_ylabel('Altitude (m)')
ax3.set_title('Altitude Profile Over Time')
ax3.grid(True)
ax3.legend()

# Add colorbar for altitude plot
sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(0, max(relative_time)))
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax3)
cbar.set_label('Time (seconds)')

plt.tight_layout()
plt.show()

# =============================== =============================== =============================== ===============================

# import pandas as pd
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import numpy as np
# import os
# from matplotlib.collections import LineCollection

# # Load the data
# log_path = os.path.join(os.path.expanduser('~'), 'zephyrDock', 'zephyrDock', 'logs', 'odometry_20250403_024114.csv')
# df = pd.read_csv(log_path)

# # Create time-normalized values for coloring (0 to 1)
# normalized_time = np.linspace(0, 1, len(df))

# # Convert to numpy arrays
# x = df['x'].values
# y = df['y'].values
# z = df['z'].values

# # Create figure with larger size
# plt.figure(figsize=(12, 8))

# # --- 2D XY Plot with Color Gradient ---
# ax1 = plt.subplot(121)
# points = np.array([x, y]).T.reshape(-1, 1, 2)
# segments = np.concatenate([points[:-1], points[1:]], axis=1)

# # Create a continuous colormap
# lc = LineCollection(segments, cmap='viridis', norm=plt.Normalize(0, 1))
# lc.set_array(normalized_time)
# lc.set_linewidth(2)
# line = ax1.add_collection(lc)

# # Add start/end markers
# ax1.scatter(x[0], y[0], c='green', s=100, label='Start', zorder=3)
# ax1.scatter(x[-1], y[-1], c='red', s=100, label='End', zorder=3)

# ax1.set_xlabel('X Position (m)')
# ax1.set_ylabel('Y Position (m)')
# ax1.set_title('XY Trajectory with Color Gradient')
# ax1.grid(True)
# ax1.legend()
# plt.colorbar(line, ax=ax1, label='Normalized Time')

# # --- 3D Plot with Color Gradient ---
# ax2 = plt.subplot(122, projection='3d')
# # Create segments for 3D plot
# for i in range(len(x)-1):
#     ax2.plot(x[i:i+2], y[i:i+2], z[i:i+2], 
#              color=plt.cm.viridis(normalized_time[i]),
#              alpha=0.8, linewidth=2)

# # Add start/end markers
# ax2.scatter(x[0], y[0], z[0], c='green', s=100, label='Start')
# ax2.scatter(x[-1], y[-1], z[-1], c='red', s=100, label='End')

# ax2.set_xlabel('X (m)')
# ax2.set_ylabel('Y (m)')
# ax2.set_zlabel('Z (m)')
# ax2.set_title('3D Flight Path with Color Gradient')
# ax2.legend()

# plt.tight_layout()
# plt.show()

# # --- Altitude Profile with Color Gradient ---
# plt.figure(figsize=(12, 5))
# for i in range(len(z)-1):
#     plt.plot([i, i+1], [z[i], z[i+1]], 
#              color=plt.cm.viridis(normalized_time[i]),
#              linewidth=2)
    
# plt.scatter(0, z[0], c='green', s=100, label='Start')
# plt.scatter(len(z)-1, z[-1], c='red', s=100, label='End')

# plt.xlabel('Time Index')
# plt.ylabel('Altitude (m)')
# plt.title('Altitude Profile with Color Gradient')
# plt.grid(True)
# plt.legend()
# cbar = plt.colorbar(plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(0, 1)))
# cbar.set_label('Normalized Time')
# plt.show()

# =============================== =============================== =============================== ===============================

# import pandas as pd
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import os
# import numpy as np

# # Load the data
# log_path = os.path.join(os.path.expanduser('~'), 'zephyrDock', 'zephyrDock', 'logs', 'odometry_20250403_024114.csv')

# try:
#     df = pd.read_csv(log_path)
#     print("Data loaded successfully!")
#     print(f"Columns available: {df.columns.tolist()}")
    
#     # Convert to numpy arrays for plotting
#     x = df['x'].to_numpy()
#     y = df['y'].to_numpy()
#     z = df['z'].to_numpy()
    
#     # Convert timestamp to relative seconds
#     if 'timestamp' in df.columns:
#         time = df['timestamp'] - df['timestamp'].min()
#     else:
#         time = np.arange(len(df))
    
#     # 2D Position Plot
#     plt.figure(figsize=(10, 6))
#     plt.plot(x, y)
#     plt.xlabel('X Position (m)')
#     plt.ylabel('Y Position (m)')
#     plt.title('Drone XY Trajectory')
#     plt.grid()
#     plt.show()
    
#     # 3D Trajectory Plot
#     fig = plt.figure(figsize=(10, 8))
#     ax = fig.add_subplot(111, projection='3d')
#     ax.plot(x, y, z)
#     ax.set_xlabel('X (m)')
#     ax.set_ylabel('Y (m)')
#     ax.set_zlabel('Z (m)')
#     ax.set_title('3D Flight Path')
#     plt.show()
    
#     # Altitude vs Time
#     plt.figure(figsize=(10, 5))
#     plt.plot(time, z)
#     plt.xlabel('Time (s)')
#     plt.ylabel('Altitude (m)')
#     plt.title('Altitude Profile')
#     plt.grid()
#     plt.show()
    
# except Exception as e:
#     print(f"Error: {e}")
#     print("Please verify:")
#     print(f"1. File exists at: {log_path}")
#     print("2. File contains columns 'x', 'y', 'z', and optionally 'timestamp'")