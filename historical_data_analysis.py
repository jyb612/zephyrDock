#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse
from datetime import datetime
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk, filedialog
from matplotlib.widgets import Button
from matplotlib.backend_bases import MouseEvent
from tkinter import simpledialog

# Constants
PX4_MODE_MAP = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
    14: "OFFBOARD", 18: "AUTO_LAND", 20: "PRECLAND"
}

class DroneLogAnalyzer:
    def __init__(self, root, log_path=None):
        self.root = root
        self.root.title("PX4 Log Analysis")
        
        # Create menu bar
        self.create_menu()
        
        if log_path:
            self.load_log(log_path)
        else:
            # Show empty window with menu to load file
            self.setup_empty_ui()

    def create_menu(self):
        """Create the menu bar with file operations"""
        menubar = tk.Menu(self.root)
        
        # File menu
        filemenu = tk.Menu(menubar, tearoff=0)
        filemenu.add_command(label="Open...", command=self.open_file)
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=self.root.quit)
        menubar.add_cascade(label="File", menu=filemenu)
        
        self.root.config(menu=menubar)

    def open_file(self):
        """Open a file dialog to select CSV file"""
        filepath = filedialog.askopenfilename(
            title="Select PX4 Log File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if filepath:
            self.load_log(filepath)

    def load_log(self, log_path):
        """Load and process the log file"""
        try:
            self.raw_df = pd.read_csv(log_path)
            self.current_file = os.path.abspath(log_path)  # Store full absolute path
            self.clean_df = self.clean_data()
            self.setup_ui()
            print(f"File loaded: {self.current_file}")  # Debug print
        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to load file: {str(e)}")
            self.setup_empty_ui()


    def setup_empty_ui(self):
        """Show empty window when no file is loaded"""
        for widget in self.root.winfo_children():
            if not isinstance(widget, tk.Menu):
                widget.destroy()
        
        label = ttk.Label(self.root, text="No log file loaded. Use File > Open to select a CSV file.")
        label.pack(expand=True)

    def clean_data(self):
        """Convert all data to numpy arrays with proper typing."""
        df = self.raw_df.copy()
        
        # Convert timestamp
        df['timestamp'] = pd.to_numeric(df['timestamp'], errors='coerce')
        df['timestamp'] -= df['timestamp'].min()
        
        # Convert modes
        df['mode_num'] = pd.to_numeric(df['mode'], errors='coerce').fillna(0).astype(int)
        df['mode_name'] = df['mode_num'].map(PX4_MODE_MAP).fillna("UNKNOWN")
        
        # Convert numerical columns
        num_cols = ['x', 'y', 'z', 'traj_x', 'traj_y', 'traj_z', 'lidar_altitude']
        for col in num_cols:
            if col in df.columns:
                df[col] = pd.to_numeric(df[col], errors='coerce').fillna(0)

        # REVERSE Z-AXIS SIGNS HERE (Add these 2 lines)
        df['z'] = -df['z']  # Makes altitude positive (up = positive)
        df['traj_z'] = -df['traj_z']  # Also reverse setpoints
        df['lidar_altitude'] = abs(df['lidar_altitude'])
        
        # Calculate errors
        for axis in ['x', 'y', 'z']:
            traj_col = f'traj_{axis}'
            if traj_col in df.columns:
                df[f'{axis}_error'] = df[traj_col] - df[axis]
        
        return df
    
    def add_filename_label(self, parent, filename):
        """Add filename label to the bottom left of a tab"""
        label_frame = ttk.Frame(parent)
        label_frame.pack(side='bottom', fill='x', padx=5, pady=5)
        
        # Truncate filename if too long
        display_name = os.path.basename(filename)
        if len(display_name) > 30:
            display_name = "..." + display_name[-27:]
        
        label = ttk.Label(
            label_frame, 
            text=f"File: {display_name}",
            relief='sunken',
            padding=(3, 1),
            font=('TkDefaultFont', 8)
        )
        label.pack(side='left', anchor='w')
    
    # Add tooltip with full path
    def show_tooltip(event):
        tooltip = tk.Toplevel(parent)
        tooltip.wm_overrideredirect(True)
        tooltip.wm_geometry(f"+{event.x_root+10}+{event.y_root+10}")
        ttk.Label(tooltip, text=filename, background='lightyellow', 
                relief='solid', borderwidth=1).pack()
        tooltip.bind("<Leave>", lambda e: tooltip.destroy())
    
        label.bind("<Enter>", show_tooltip)

    def setup_ui(self):
            """Create tabbed interface with visible filename display"""
            # Clear existing widgets
            for widget in self.root.winfo_children():
                if not isinstance(widget, tk.Menu):
                    widget.destroy()

            # Main container with border for debugging
            main_frame = ttk.Frame(self.root, borderwidth=2, relief='groove')
            main_frame.pack(fill='both', expand=True, padx=5, pady=5)

            # Status bar frame with bright background for visibility
            status_frame = ttk.Frame(main_frame, height=25, style='Status.TFrame')
            status_frame.pack(fill='x', side='bottom', pady=(5,0))

            # Configure style for visibility
            style = ttk.Style()
            style.configure('Status.TFrame', background='#f0f0f0')
            style.configure('Status.TLabel', background='#f0f0f0', font=('TkDefaultFont', 9))

            # Add filename label - now with more prominent styling
            if hasattr(self, 'current_file'):
                short_name = os.path.basename(self.current_file)
                status_label = ttk.Label(
                    status_frame,
                    text=f"Loaded: {short_name}",
                    style='Status.TLabel',
                    relief='ridge',
                    padding=(5,2),
                    anchor='w'
                )
                status_label.pack(fill='x', padx=5, pady=2)

                # Add temporary debug border
                status_label.config(borderwidth=1, relief='solid')

                # Tooltip for full path
                def show_tooltip(event):
                    tooltip = tk.Toplevel(self.root)
                    tooltip.wm_overrideredirect(True)
                    tooltip.wm_geometry(f"+{event.x_root+10}+{event.y_root+10}")
                    label = ttk.Label(tooltip, text=self.current_file, 
                                    background='lightyellow', 
                                    relief='solid', 
                                    padding=5)
                    label.pack()
                    tooltip.bind("<Leave>", lambda e: tooltip.destroy())
                
                status_label.bind("<Enter>", show_tooltip)
            else:
                # Debug placeholder
                ttk.Label(status_frame, text="No file loaded", style='Status.TLabel').pack()

            # Notebook for tabs - placed above status bar
            notebook = ttk.Notebook(main_frame)
            notebook.pack(fill='both', expand=True)

            # Rest of your tab creation code...
            tabs = [
                ("XY Trajectory", self.create_xy_trajectory_tab),  # Add this line
                ("Altitude", self.create_altitude_tab),
                ("Errors", self.create_errors_tab),
                ("Modes", self.create_modes_tab)
            ]
            
            if 'servo pos' in self.clean_df.columns:
                servo_data = pd.to_numeric(self.clean_df['servo pos'], errors='coerce')
                if not servo_data.isna().all():
                    tabs.append(("Servo", self.create_servo_tab))
            
            for name, func in tabs:
                tab = ttk.Frame(notebook)
                notebook.add(tab, text=name)
                try:
                    func(tab)
                except Exception as e:
                    ttk.Label(tab, text=f"Error: {str(e)}").pack()

            # Force UI update
            self.root.update_idletasks()
            print("UI refreshed - status frame should be visible")

    
    def create_altitude_tab(self, parent):
        """Altitude tracking using pure numpy arrays."""
        fig = plt.Figure(figsize=(10, 8))
        
        # Convert to numpy explicitly
        mask = ~self.clean_df[['timestamp', 'z', 'traj_z', 'lidar_altitude']].isna().any(axis=1)
        data = self.clean_df.loc[mask, ['timestamp', 'z', 'traj_z', 'lidar_altitude']]
        t = data['timestamp'].to_numpy()
        z = data['z'].to_numpy()
        z_sp = data['traj_z'].to_numpy()
        lidar = data['lidar_altitude'].to_numpy()

        # Altitude plot
        ax1 = fig.add_subplot(2, 1, 1)
        ax1.plot(t, z, label='Actual')
        ax1.plot(t, z_sp, '--', label='Setpoint')
        ax1.set_ylabel('Altitude (m, NED)')
        ax1.legend()
        ax1.grid()

        # Lidar plot
        ax2 = fig.add_subplot(2, 1, 2)
        ax2.plot(t, lidar, 'g', label='Lidar')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Height AGL (m)')
        ax2.legend()
        ax2.grid()
        self.enable_phase_marking(fig, ax1)
        self.enable_phase_marking(fig, ax2)

        fig.original_limits = [(ax.get_xlim(), ax.get_ylim()) 
                          for ax in fig.get_axes()]
    
        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self.add_zoom_controls(fig, parent)
        canvas.draw()

    def create_errors_tab(self, parent):
        """Error analysis using pure numpy arrays."""
        fig = plt.Figure(figsize=(10, 8))
        
        # Convert to numpy explicitly
        mask = ~self.clean_df[['timestamp', 'x_error', 'y_error', 'z_error']].isna().any(axis=1)
        data = self.clean_df.loc[mask, ['timestamp', 'x_error', 'y_error', 'z_error']]
        t = data['timestamp'].to_numpy()
        x_err = data['x_error'].to_numpy()
        y_err = data['y_error'].to_numpy()
        z_err = data['z_error'].to_numpy()

        # XY Errors
        ax1 = fig.add_subplot(2, 1, 1)
        ax1.plot(t, x_err, label='X Error')
        ax1.plot(t, y_err, label='Y Error')
        ax1.set_ylabel('Error (m)')
        ax1.legend()
        ax1.grid()

        # Z Error
        ax2 = fig.add_subplot(2, 1, 2)
        ax2.plot(t, z_err, 'r', label='Z Error')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error (m)')
        ax2.legend()
        ax2.grid()
        self.enable_phase_marking(fig, ax1)
        self.enable_phase_marking(fig, ax2) 

        fig.original_limits = [(ax.get_xlim(), ax.get_ylim()) 
                          for ax in fig.get_axes()]
    
        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self.add_zoom_controls(fig, parent)
        canvas.draw()

    def create_modes_tab(self, parent):
        """Mode transitions with proper type handling."""
        fig = plt.Figure(figsize=(10, 4))
        ax = fig.add_subplot(1, 1, 1)
        
        # Get clean data
        mask = ~self.clean_df[['timestamp', 'mode_num', 'mode_name']].isna().any(axis=1)
        data = self.clean_df.loc[mask, ['timestamp', 'mode_num', 'mode_name']]
        
        # Plot each mode
        mode_labels = list(PX4_MODE_MAP.values())
        for mode in data['mode_name'].unique():
            if mode in mode_labels:
                mask = data['mode_name'] == mode
                ax.scatter(data.loc[mask, 'timestamp'].to_numpy(),
                          data.loc[mask, 'mode_num'].to_numpy(),
                          label=mode, s=10)
        
        ax.set_xlabel('Time (s)')
        ax.set_yticks(range(len(mode_labels)))
        ax.set_yticklabels(mode_labels)
        ax.grid()
        ax.legend()
        self.enable_phase_marking(fig, ax)

        fig.original_limits = [(ax.get_xlim(), ax.get_ylim()) 
                          for ax in fig.get_axes()]
    
        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self.add_zoom_controls(fig, parent)
        canvas.draw()

    def create_servo_tab(self, parent):
        """Servo analysis using pure numpy arrays."""
        fig = plt.Figure(figsize=(10, 4))
        ax = fig.add_subplot(1, 1, 1)
        
        # Convert to numpy explicitly
        mask = ~self.clean_df[['timestamp', 'servo pos']].isna().any(axis=1)
        data = self.clean_df.loc[mask, ['timestamp', 'servo pos']]
        t = data['timestamp'].to_numpy()
        servo = data['servo pos'].to_numpy()

        ax.plot(t, servo, 'purple')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Servo Position')
        ax.grid()
        self.enable_phase_marking(fig, ax)

        fig.original_limits = [(ax.get_xlim(), ax.get_ylim()) 
                          for ax in fig.get_axes()]
    
        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self.add_zoom_controls(fig, parent)
        canvas.draw()
    
    def create_xy_trajectory_tab(self, parent):
        """Create X-Y plane trajectory visualization with time slider"""
        rotate_90 = False  # Set True to rotate all data 90° counterclockwise

        fig = plt.Figure(figsize=(10, 8))
        fig.subplots_adjust(bottom=0.2)
        ax = fig.add_subplot(1, 1, 1)

        # Get clean data
        mask = ~self.clean_df[['timestamp', 'x', 'y', 'traj_x', 'traj_y']].isna().any(axis=1)
        data = self.clean_df.loc[mask, ['timestamp', 'x', 'y', 'traj_x', 'traj_y']]
        t = data['timestamp'].to_numpy()
        x = data['x'].to_numpy()
        y = data['y'].to_numpy()
        x_sp = data['traj_x'].to_numpy()
        y_sp = data['traj_y'].to_numpy()

        # Optional 90° counterclockwise rotation
        if rotate_90:
            x, y = -y, x
            x_sp, y_sp = -y_sp, x_sp

        # Plot trajectories
        ax.plot(x_sp, y_sp, 'b--', label='Setpoint Trajectory', alpha=0.5)
        ax.plot(x, y, 'g-', label='Actual Trajectory', alpha=0.7)

        # Current position markers
        current_pos, = ax.plot([], [], 'ro', markersize=8, label='Current Position')
        current_sp, = ax.plot([], [], 'bo', markersize=8, label='Current Setpoint')

        # Flip X-axis direction so positive points left
        ax.invert_xaxis()

        # Axes and layout
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(True)
        ax.axhline(0, color='black', linewidth=0.5)
        ax.axvline(0, color='black', linewidth=0.5)
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.legend()

        # Store data for slider update
        fig.xy_data = (t, x, y, x_sp, y_sp)
        fig.current_pos = current_pos
        fig.current_sp = current_sp

        # Time slider
        slider_ax = fig.add_axes([0.2, 0.05, 0.6, 0.03])
        time_slider = plt.Slider(
            slider_ax, 'Time (s)', t[0], t[-1],
            valinit=t[0], valstep=(t[-1] - t[0]) / 1000
        )

        def update(val):
            """Update markers based on time slider"""
            time = time_slider.val
            idx = np.argmin(np.abs(t - time))
            fig.current_pos.set_data([x[idx]], [y[idx]])
            fig.current_sp.set_data([x_sp[idx]], [y_sp[idx]])
            fig.canvas.draw_idle()

        time_slider.on_changed(update)
        fig.time_slider = time_slider

        # Zoom controls and draw
        fig.original_limits = [(ax.get_xlim(), ax.get_ylim())]
        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self.add_zoom_controls(fig, parent)
        canvas.draw()



    def enable_phase_marking(self, fig, ax):
        """Enable interactive phase marking with working removal"""
        if not hasattr(fig, 'phase_markers'):
            fig.phase_markers = []  # Stores (time, label, line, text) tuples
            
        def onclick(event):
            if event.inaxes == ax:
                if event.dblclick:  # Add marker
                    time = event.xdata
                    label = simpledialog.askstring("Phase Marker", 
                                                f"Time: {time:.2f}s\nEnter phase name:",
                                                parent=self.root)
                    if label:
                        # Create the visual elements
                        line = ax.axvline(x=time, color='gray', linestyle='--', alpha=0.7)
                        text = ax.text(time, ax.get_ylim()[1]*0.95, label,
                                    rotation=90, va='top', ha='right',
                                    backgroundcolor='white', alpha=0.8)
                        # Store references to remove later
                        fig.phase_markers.append((time, label, line, text))
                        fig.canvas.draw_idle()
                        
                elif event.button == 3:  # Right-click remove
                    if fig.phase_markers:
                        # Find closest marker
                        times = [m[0] for m in fig.phase_markers]
                        idx = np.argmin(np.abs(np.array(times) - event.xdata))
                        
                        # Remove visual elements
                        fig.phase_markers[idx][2].remove()  # Remove line
                        fig.phase_markers[idx][3].remove()  # Remove text
                        
                        # Remove from storage
                        fig.phase_markers.pop(idx)
                        fig.canvas.draw_idle()
        
        fig.canvas.mpl_connect('button_press_event', onclick)

    def add_phase_marker(self, ax, time, label):
        """Add a single phase marker to plot"""
        ax.axvline(x=time, color='gray', linestyle='--', alpha=0.7)
        ax.text(time, ax.get_ylim()[1]*0.95, label, 
            rotation=90, va='top', ha='right',
            backgroundcolor='white', alpha=0.8)

    def redraw_phase_markers(self, fig, ax):
        """Redraw all phase markers after removal"""
        # Clear existing vertical lines
        for artist in ax.lines + ax.texts:
            if hasattr(artist, '_is_phase_marker'):
                artist.remove()
        
        # Redraw all markers
        for time, label in fig.phase_markers:
            line = ax.axvline(x=time, color='gray', linestyle='--', alpha=0.7)
            text = ax.text(time, ax.get_ylim()[1]*0.95, label,
                        rotation=90, va='top', ha='right',
                        backgroundcolor='white', alpha=0.8)
            # Tag markers for easy removal
            line._is_phase_marker = True
            text._is_phase_marker = True
    
    def add_zoom_controls(self, fig, parent):
        # Store zoom state
        fig.zoom_stack = []
        """Add zoom controls in top-right corner"""
        # Create a frame for controls in the top-right
        control_frame = ttk.Frame(parent)
        control_frame.place(relx=1.0, rely=0.0, anchor='ne', x=-10, y=10)  # Top-right with padding
        
        # Make buttons smaller and more compact
        style = ttk.Style()
        style.configure('Zoom.TButton', padding=2, font=('Helvetica', 8))
        
        # Add buttons in horizontal layout
        ttk.Button(control_frame, text="+", style='Zoom.TButton',
                command=lambda: self.zoom_axes(2.0, fig)).pack(side='left', padx=2)
        ttk.Button(control_frame, text="-", style='Zoom.TButton',
                command=lambda: self.zoom_axes(0.5, fig)).pack(side='left', padx=2)
        ttk.Button(control_frame, text="↻", style='Zoom.TButton',
                command=lambda: self.reset_zoom(fig)).pack(side='left', padx=2)
        
        # Keep scroll zoom functionality
        def on_scroll(event):
            if event.inaxes:
                self.zoom_axes(1.1 if event.button == 'up' else 0.9, 
                            fig, event.x, event.y, event.inaxes)
        fig.canvas.mpl_connect('scroll_event', on_scroll)

    def zoom_axes(self, factor, fig, x=None, y=None, ax=None):
        """Zoom centered around cursor position or axes center"""
        for current_ax in fig.get_axes():
            if ax and current_ax != ax:
                continue  # Only zoom the axes the mouse is over
                
            xlim = current_ax.get_xlim()
            ylim = current_ax.get_ylim()
            
            if x is not None and y is not None:
                # Convert screen coordinates to data coordinates
                xdata, ydata = current_ax.transData.inverted().transform_point((x, y))
            else:
                # Center if no cursor position
                xdata = np.mean(xlim)
                ydata = np.mean(ylim)
            
            # Calculate new limits
            new_width = (xlim[1] - xlim[0]) / factor
            new_height = (ylim[1] - ylim[0]) / factor
            
            current_ax.set_xlim([xdata - new_width/2, xdata + new_width/2])
            current_ax.set_ylim([ydata - new_height/2, ydata + new_height/2])
        
        fig.zoom_stack.append((current_ax.get_xlim(), current_ax.get_ylim()))
        fig.canvas.draw_idle()

    def reset_zoom(self, fig):
        """Reset to original view using stored limits"""
        if hasattr(fig, 'original_limits'):
            for ax, (xlim, ylim) in zip(fig.get_axes(), fig.original_limits):
                ax.set_xlim(xlim)
                ax.set_ylim(ylim)
            fig.canvas.draw_idle()

def main():
    root = tk.Tk()
    
    # Check for command line argument
    parser = argparse.ArgumentParser(description='PX4 Log Analyzer')
    parser.add_argument('log_file', nargs='?', help='Path to CSV log file')
    args = parser.parse_args()
    
    if args.log_file and os.path.exists(args.log_file):
        app = DroneLogAnalyzer(root, args.log_file)
    else:
        app = DroneLogAnalyzer(root)
    
    root.mainloop()

if __name__ == "__main__":
    main()

# =============================== =============================== =============================== ===============================

# #!/usr/bin/env python3
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np
# import os
# import argparse
# from datetime import datetime
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# import tkinter as tk
# from tkinter import ttk
# from matplotlib.widgets import Button
# from matplotlib.backend_bases import MouseEvent
# from tkinter import simpledialog

# # Constants
# PX4_MODE_MAP = {
#     0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
#     14: "OFFBOARD", 18: "AUTO_LAND", 20: "PRECLAND"
# }

# class DroneLogAnalyzer:
#     def __init__(self, log_path):
#         self.raw_df = pd.read_csv(log_path)
#         self.clean_df = self.clean_data()
#         self.root = tk.Tk()
#         self.root.title("PX4 Log Analysis")
#         self.setup_ui()

#     def clean_data(self):
#         """Convert all data to numpy arrays with proper typing."""
#         df = self.raw_df.copy()
        
#         # Convert timestamp
#         df['timestamp'] = pd.to_numeric(df['timestamp'], errors='coerce')
#         df['timestamp'] -= df['timestamp'].min()
        
#         # Convert modes
#         df['mode_num'] = pd.to_numeric(df['mode'], errors='coerce').fillna(0).astype(int)
#         df['mode_name'] = df['mode_num'].map(PX4_MODE_MAP).fillna("UNKNOWN")
        
#         # Convert numerical columns
#         num_cols = ['x', 'y', 'z', 'traj_x', 'traj_y', 'traj_z', 'lidar_altitude']
#         for col in num_cols:
#             if col in df.columns:
#                 df[col] = pd.to_numeric(df[col], errors='coerce').fillna(0)

#         # REVERSE Z-AXIS SIGNS HERE (Add these 2 lines)
#         df['z'] = -df['z']  # Makes altitude positive (up = positive)
#         df['traj_z'] = -df['traj_z']  # Also reverse setpoints
#         df['lidar_altitude'] = abs(df['lidar_altitude'])
        
#         # Calculate errors
#         for axis in ['x', 'y', 'z']:
#             traj_col = f'traj_{axis}'
#             if traj_col in df.columns:
#                 df[f'{axis}_error'] = df[traj_col] - df[axis]
        
#         return df

#     def setup_ui(self):
#         """Create tabbed interface with proper data conversion."""
#         notebook = ttk.Notebook(self.root)
        
#         # Create tabs
#         tabs = [
#             ("Altitude", self.create_altitude_tab),
#             ("Errors", self.create_errors_tab),
#             ("Modes", self.create_modes_tab)
#         ]
        
#         # Add Servo tab if data exists
#         if 'servo pos' in self.clean_df.columns:
#             servo_data = pd.to_numeric(self.clean_df['servo pos'], errors='coerce')
#             if not servo_data.isna().all():
#                 tabs.append(("Servo", self.create_servo_tab))
        
#         for name, func in tabs:
#             tab = ttk.Frame(notebook)
#             notebook.add(tab, text=name)
#             try:
#                 func(tab)
#             except Exception as e:
#                 print(f"Warning: Could not create {name} tab - {str(e)}")
#                 label = ttk.Label(tab, text=f"Data not available for {name}")
#                 label.pack()
        
#         notebook.pack(expand=True, fill="both")
#         self.root.mainloop()

#     def create_altitude_tab(self, parent):
#         """Altitude tracking using pure numpy arrays."""
#         fig = plt.Figure(figsize=(10, 8))
        
#         # Convert to numpy explicitly
#         mask = ~self.clean_df[['timestamp', 'z', 'traj_z', 'lidar_altitude']].isna().any(axis=1)
#         data = self.clean_df.loc[mask, ['timestamp', 'z', 'traj_z', 'lidar_altitude']]
#         t = data['timestamp'].to_numpy()
#         z = data['z'].to_numpy()
#         z_sp = data['traj_z'].to_numpy()
#         lidar = data['lidar_altitude'].to_numpy()

#         # # Define your phases (time, label)
#         # phases = [
#         #     (10.5, "Takeoff"),
#         #     (45.2, "Cruise"),
#         #     (85.7, "Landing")
#         # ]

#         # self.add_phase_marker(ax1, phases)
#         # Add to BOTH subplots (or just one if you prefer)
        
#         # def add_phase_markers(self, ax, phases):
#         #     """Add vertical phase markers to plots"""
#         #     for time, label in phases:
#         #         ax.axvline(x=time, color='gray', linestyle='--', alpha=0.7)
#         #         ax.text(time, ax.get_ylim()[1]*0.95, label, 
#         #             rotation=90, va='top', ha='right',
#         #             backgroundcolor='white', alpha=0.8)
            


#         # self.add_phase_marker(ax1, 10.5, "Takeoff")
#         # fig.phase_markers.append((10.5, "Takeoff"))

#         # Altitude plot
#         ax1 = fig.add_subplot(2, 1, 1)
#         ax1.plot(t, z, label='Actual')
#         ax1.plot(t, z_sp, '--', label='Setpoint')
#         # self.enable_phase_marking(fig, ax2)
#         ax1.set_ylabel('Altitude (m, NED)')
#         ax1.legend()
#         ax1.grid()

#         # Lidar plot
#         ax2 = fig.add_subplot(2, 1, 2)
#         ax2.plot(t, lidar, 'g', label='Lidar')
#         ax2.set_xlabel('Time (s)')
#         ax2.set_ylabel('Height AGL (m)')
#         ax2.legend()
#         ax2.grid()
#         self.enable_phase_marking(fig, ax1)
#         self.enable_phase_marking(fig, ax2)

#         fig.original_limits = [(ax.get_xlim(), ax.get_ylim()) 
#                           for ax in fig.get_axes()]
    
#         canvas = FigureCanvasTkAgg(fig, master=parent)
#         canvas.get_tk_widget().pack(fill="both", expand=True)
#         self.add_zoom_controls(fig, parent)
#         canvas.draw()

#     def create_errors_tab(self, parent):
#         """Error analysis using pure numpy arrays."""
#         fig = plt.Figure(figsize=(10, 8))
        
#         # Convert to numpy explicitly
#         mask = ~self.clean_df[['timestamp', 'x_error', 'y_error', 'z_error']].isna().any(axis=1)
#         data = self.clean_df.loc[mask, ['timestamp', 'x_error', 'y_error', 'z_error']]
#         t = data['timestamp'].to_numpy()
#         x_err = data['x_error'].to_numpy()
#         y_err = data['y_error'].to_numpy()
#         z_err = data['z_error'].to_numpy()

#         # XY Errors
#         ax1 = fig.add_subplot(2, 1, 1)
#         ax1.plot(t, x_err, label='X Error')
#         ax1.plot(t, y_err, label='Y Error')
#         ax1.set_ylabel('Error (m)')
#         ax1.legend()
#         ax1.grid()

#         # Z Error
#         ax2 = fig.add_subplot(2, 1, 2)
#         ax2.plot(t, z_err, 'r', label='Z Error')
#         ax2.set_xlabel('Time (s)')
#         ax2.set_ylabel('Error (m)')
#         ax2.legend()
#         ax2.grid()
#         self.enable_phase_marking(fig, ax1)
#         self.enable_phase_marking(fig, ax2) 

#         fig.original_limits = [(ax.get_xlim(), ax.get_ylim()) 
#                           for ax in fig.get_axes()]
    
#         canvas = FigureCanvasTkAgg(fig, master=parent)
#         canvas.get_tk_widget().pack(fill="both", expand=True)
#         self.add_zoom_controls(fig, parent)
#         canvas.draw()

#     def create_modes_tab(self, parent):
#         """Mode transitions with proper type handling."""
#         fig = plt.Figure(figsize=(10, 4))
#         ax = fig.add_subplot(1, 1, 1)
        
#         # Get clean data
#         mask = ~self.clean_df[['timestamp', 'mode_num', 'mode_name']].isna().any(axis=1)
#         data = self.clean_df.loc[mask, ['timestamp', 'mode_num', 'mode_name']]
        
#         # Plot each mode
#         mode_labels = list(PX4_MODE_MAP.values())
#         for mode in data['mode_name'].unique():
#             if mode in mode_labels:
#                 mask = data['mode_name'] == mode
#                 ax.scatter(data.loc[mask, 'timestamp'].to_numpy(),
#                           data.loc[mask, 'mode_num'].to_numpy(),
#                           label=mode, s=10)
        
#         ax.set_xlabel('Time (s)')
#         ax.set_yticks(range(len(mode_labels)))
#         ax.set_yticklabels(mode_labels)
#         ax.grid()
#         ax.legend()
#         self.enable_phase_marking(fig, ax1)
#         self.enable_phase_marking(fig, ax2)

#         fig.original_limits = [(ax.get_xlim(), ax.get_ylim()) 
#                           for ax in fig.get_axes()]
    
#         canvas = FigureCanvasTkAgg(fig, master=parent)
#         canvas.get_tk_widget().pack(fill="both", expand=True)
#         self.add_zoom_controls(fig, parent)
#         canvas.draw()

#     def create_servo_tab(self, parent):
#         """Servo analysis using pure numpy arrays."""
#         fig = plt.Figure(figsize=(10, 4))
#         ax = fig.add_subplot(1, 1, 1)
        
#         # Convert to numpy explicitly
#         mask = ~self.clean_df[['timestamp', 'servo pos']].isna().any(axis=1)
#         data = self.clean_df.loc[mask, ['timestamp', 'servo pos']]
#         t = data['timestamp'].to_numpy()
#         servo = data['servo pos'].to_numpy()

#         ax.plot(t, servo, 'purple')
#         ax.set_xlabel('Time (s)')
#         ax.set_ylabel('Servo Position')
#         ax.grid()
#         self.enable_phase_marking(fig, ax1)
#         self.enable_phase_marking(fig, ax2)

#         fig.original_limits = [(ax.get_xlim(), ax.get_ylim()) 
#                           for ax in fig.get_axes()]
    
#         canvas = FigureCanvasTkAgg(fig, master=parent)
#         canvas.get_tk_widget().pack(fill="both", expand=True)
#         self.add_zoom_controls(fig, parent)
#         canvas.draw()
    
#     def enable_phase_marking(self, fig, ax):
#         """Enable interactive phase marking with working removal"""
#         if not hasattr(fig, 'phase_markers'):
#             fig.phase_markers = []  # Stores (time, label, line, text) tuples
            
#         def onclick(event):
#             if event.inaxes == ax:
#                 if event.dblclick:  # Add marker
#                     time = event.xdata
#                     label = simpledialog.askstring("Phase Marker", 
#                                                 f"Time: {time:.2f}s\nEnter phase name:",
#                                                 parent=self.root)
#                     if label:
#                         # Create the visual elements
#                         line = ax.axvline(x=time, color='gray', linestyle='--', alpha=0.7)
#                         text = ax.text(time, ax.get_ylim()[1]*0.95, label,
#                                     rotation=90, va='top', ha='right',
#                                     backgroundcolor='white', alpha=0.8)
#                         # Store references to remove later
#                         fig.phase_markers.append((time, label, line, text))
#                         fig.canvas.draw_idle()
                        
#                 elif event.button == 3:  # Right-click remove
#                     if fig.phase_markers:
#                         # Find closest marker
#                         times = [m[0] for m in fig.phase_markers]
#                         idx = np.argmin(np.abs(np.array(times) - event.xdata))
                        
#                         # Remove visual elements
#                         fig.phase_markers[idx][2].remove()  # Remove line
#                         fig.phase_markers[idx][3].remove()  # Remove text
                        
#                         # Remove from storage
#                         fig.phase_markers.pop(idx)
#                         fig.canvas.draw_idle()
        
#         fig.canvas.mpl_connect('button_press_event', onclick)

#     def add_phase_marker(self, ax, time, label):
#         """Add a single phase marker to plot"""
#         ax.axvline(x=time, color='gray', linestyle='--', alpha=0.7)
#         ax.text(time, ax.get_ylim()[1]*0.95, label, 
#             rotation=90, va='top', ha='right',
#             backgroundcolor='white', alpha=0.8)

#     def redraw_phase_markers(self, fig, ax):
#         """Redraw all phase markers after removal"""
#         # Clear existing vertical lines
#         for artist in ax.lines + ax.texts:
#             if hasattr(artist, '_is_phase_marker'):
#                 artist.remove()
        
#         # Redraw all markers
#         for time, label in fig.phase_markers:
#             line = ax.axvline(x=time, color='gray', linestyle='--', alpha=0.7)
#             text = ax.text(time, ax.get_ylim()[1]*0.95, label,
#                         rotation=90, va='top', ha='right',
#                         backgroundcolor='white', alpha=0.8)
#             # Tag markers for easy removal
#             line._is_phase_marker = True
#             text._is_phase_marker = True
    
#     def add_zoom_controls(self, fig, parent):
#         # Store zoom state
#         fig.zoom_stack = []
#         """Add zoom controls in top-right corner"""
#         # Create a frame for controls in the top-right
#         control_frame = ttk.Frame(parent)
#         control_frame.place(relx=1.0, rely=0.0, anchor='ne', x=-10, y=10)  # Top-right with padding
        
#         # Make buttons smaller and more compact
#         style = ttk.Style()
#         style.configure('Zoom.TButton', padding=2, font=('Helvetica', 8))
        
#         # Add buttons in horizontal layout
#         ttk.Button(control_frame, text="+", style='Zoom.TButton',
#                 command=lambda: self.zoom_axes(2.0, fig)).pack(side='left', padx=2)
#         ttk.Button(control_frame, text="-", style='Zoom.TButton',
#                 command=lambda: self.zoom_axes(0.5, fig)).pack(side='left', padx=2)
#         ttk.Button(control_frame, text="↻", style='Zoom.TButton',
#                 command=lambda: self.reset_zoom(fig)).pack(side='left', padx=2)
        
#         # Keep scroll zoom functionality
#         def on_scroll(event):
#             if event.inaxes:
#                 self.zoom_axes(1.1 if event.button == 'up' else 0.9, 
#                             fig, event.x, event.y, event.inaxes)
#         fig.canvas.mpl_connect('scroll_event', on_scroll)

#     def zoom_axes(self, factor, fig, x=None, y=None, ax=None):
#         """Zoom centered around cursor position or axes center"""
#         for current_ax in fig.get_axes():
#             if ax and current_ax != ax:
#                 continue  # Only zoom the axes the mouse is over
                
#             xlim = current_ax.get_xlim()
#             ylim = current_ax.get_ylim()
            
#             if x is not None and y is not None:
#                 # Convert screen coordinates to data coordinates
#                 xdata, ydata = current_ax.transData.inverted().transform_point((x, y))
#             else:
#                 # Center if no cursor position
#                 xdata = np.mean(xlim)
#                 ydata = np.mean(ylim)
            
#             # Calculate new limits
#             new_width = (xlim[1] - xlim[0]) / factor
#             new_height = (ylim[1] - ylim[0]) / factor
            
#             current_ax.set_xlim([xdata - new_width/2, xdata + new_width/2])
#             current_ax.set_ylim([ydata - new_height/2, ydata + new_height/2])
        
#         fig.zoom_stack.append((current_ax.get_xlim(), current_ax.get_ylim()))
#         fig.canvas.draw_idle()

#     def reset_zoom(self, fig):
#         """Reset to original view using stored limits"""
#         if hasattr(fig, 'original_limits'):
#             for ax, (xlim, ylim) in zip(fig.get_axes(), fig.original_limits):
#                 ax.set_xlim(xlim)
#                 ax.set_ylim(ylim)
#             fig.canvas.draw_idle()

# def main():
#     parser = argparse.ArgumentParser(description='PX4 Log Analyzer')
#     parser.add_argument('log_file', help='Path to CSV log file')
#     args = parser.parse_args()
    
#     if not os.path.exists(args.log_file):
#         print(f"Error: File not found - {args.log_file}")
#         return
    
#     DroneLogAnalyzer(args.log_file)

# if __name__ == "__main__":
#     main()
    
# =============================== =============================== =============================== ===============================

# #!/usr/bin/env python3
# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np
# import os
# import argparse
# from datetime import datetime
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# import tkinter as tk
# from tkinter import ttk
# from matplotlib.widgets import Button

# # Constants
# PX4_MODE_MAP = {
#     0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
#     14: "OFFBOARD", 18: "AUTO_LAND", 20: "PRECLAND"
# }

# class DroneLogAnalyzer:
#     def __init__(self, log_path):
#         self.raw_df = pd.read_csv(log_path)
#         self.clean_df = self.clean_data()
#         self.root = tk.Tk()
#         self.root.title("PX4 Log Analysis")
#         self.setup_ui()

#     def clean_data(self):
#         """Convert all data to numpy arrays with proper typing."""
#         df = self.raw_df.copy()
        
#         # Convert timestamp
#         df['timestamp'] = pd.to_numeric(df['timestamp'], errors='coerce')
#         df['timestamp'] -= df['timestamp'].min()
        
#         # Convert modes
#         df['mode_num'] = pd.to_numeric(df['mode'], errors='coerce').fillna(0).astype(int)
#         df['mode_name'] = df['mode_num'].map(PX4_MODE_MAP).fillna("UNKNOWN")
        
#         # Convert numerical columns
#         num_cols = ['x', 'y', 'z', 'traj_x', 'traj_y', 'traj_z', 'lidar_altitude']
#         for col in num_cols:
#             if col in df.columns:
#                 df[col] = pd.to_numeric(df[col], errors='coerce').fillna(0)

#         # REVERSE Z-AXIS SIGNS HERE (Add these 2 lines)
#         df['z'] = -df['z']  # Makes altitude positive (up = positive)
#         df['traj_z'] = -df['traj_z']  # Also reverse setpoints
#         df['lidar_altitude'] = abs(df['lidar_altitude'])
        
#         # Calculate errors
#         for axis in ['x', 'y', 'z']:
#             traj_col = f'traj_{axis}'
#             if traj_col in df.columns:
#                 df[f'{axis}_error'] = df[traj_col] - df[axis]
        
#         return df

#     def setup_ui(self):
#         """Create tabbed interface with proper data conversion."""
#         notebook = ttk.Notebook(self.root)
        
#         # Create tabs
#         tabs = [
#             ("Altitude", self.create_altitude_tab),
#             ("Errors", self.create_errors_tab),
#             ("Modes", self.create_modes_tab)
#         ]
        
#         # Add Servo tab if data exists
#         if 'servo pos' in self.clean_df.columns:
#             servo_data = pd.to_numeric(self.clean_df['servo pos'], errors='coerce')
#             if not servo_data.isna().all():
#                 tabs.append(("Servo", self.create_servo_tab))
        
#         for name, func in tabs:
#             tab = ttk.Frame(notebook)
#             notebook.add(tab, text=name)
#             try:
#                 func(tab)
#             except Exception as e:
#                 print(f"Warning: Could not create {name} tab - {str(e)}")
#                 label = ttk.Label(tab, text=f"Data not available for {name}")
#                 label.pack()
        
#         notebook.pack(expand=True, fill="both")
#         self.root.mainloop()

#     def create_altitude_tab(self, parent):
#         """Altitude tracking using pure numpy arrays."""
#         fig = plt.Figure(figsize=(10, 8))
        
#         # Convert to numpy explicitly
#         mask = ~self.clean_df[['timestamp', 'z', 'traj_z', 'lidar_altitude']].isna().any(axis=1)
#         data = self.clean_df.loc[mask, ['timestamp', 'z', 'traj_z', 'lidar_altitude']]
#         t = data['timestamp'].to_numpy()
#         z = data['z'].to_numpy()
#         z_sp = data['traj_z'].to_numpy()
#         lidar = data['lidar_altitude'].to_numpy()

#         # Altitude plot
#         ax1 = fig.add_subplot(2, 1, 1)
#         ax1.plot(t, z, label='Actual')
#         ax1.plot(t, z_sp, '--', label='Setpoint')
#         ax1.set_ylabel('Altitude (m, NED)')
#         ax1.legend()
#         ax1.grid()

#         # Lidar plot
#         ax2 = fig.add_subplot(2, 1, 2)
#         ax2.plot(t, lidar, 'g', label='Lidar')
#         ax2.set_xlabel('Time (s)')
#         ax2.set_ylabel('Height AGL (m)')
#         ax2.legend()
#         ax2.grid()

#         canvas = FigureCanvasTkAgg(fig, master=parent)
#         canvas.get_tk_widget().pack(fill="both", expand=True)
#         self.add_zoom_controls(fig, parent)
#         canvas.draw()

#     def create_errors_tab(self, parent):
#         """Error analysis using pure numpy arrays."""
#         fig = plt.Figure(figsize=(10, 8))
        
#         # Convert to numpy explicitly
#         mask = ~self.clean_df[['timestamp', 'x_error', 'y_error', 'z_error']].isna().any(axis=1)
#         data = self.clean_df.loc[mask, ['timestamp', 'x_error', 'y_error', 'z_error']]
#         t = data['timestamp'].to_numpy()
#         x_err = data['x_error'].to_numpy()
#         y_err = data['y_error'].to_numpy()
#         z_err = data['z_error'].to_numpy()

#         # XY Errors
#         ax1 = fig.add_subplot(2, 1, 1)
#         ax1.plot(t, x_err, label='X Error')
#         ax1.plot(t, y_err, label='Y Error')
#         ax1.set_ylabel('Error (m)')
#         ax1.legend()
#         ax1.grid()

#         # Z Error
#         ax2 = fig.add_subplot(2, 1, 2)
#         ax2.plot(t, z_err, 'r', label='Z Error')
#         ax2.set_xlabel('Time (s)')
#         ax2.set_ylabel('Error (m)')
#         ax2.legend()
#         ax2.grid()

#         canvas = FigureCanvasTkAgg(fig, master=parent)
#         canvas.get_tk_widget().pack(fill="both", expand=True)
#         self.add_zoom_controls(fig, parent)
#         canvas.draw()

#     def create_modes_tab(self, parent):
#         """Mode transitions with proper type handling."""
#         fig = plt.Figure(figsize=(10, 4))
#         ax = fig.add_subplot(1, 1, 1)
        
#         # Get clean data
#         mask = ~self.clean_df[['timestamp', 'mode_num', 'mode_name']].isna().any(axis=1)
#         data = self.clean_df.loc[mask, ['timestamp', 'mode_num', 'mode_name']]
        
#         # Plot each mode
#         mode_labels = list(PX4_MODE_MAP.values())
#         for mode in data['mode_name'].unique():
#             if mode in mode_labels:
#                 mask = data['mode_name'] == mode
#                 ax.scatter(data.loc[mask, 'timestamp'].to_numpy(),
#                           data.loc[mask, 'mode_num'].to_numpy(),
#                           label=mode, s=10)
        
#         ax.set_xlabel('Time (s)')
#         ax.set_yticks(range(len(mode_labels)))
#         ax.set_yticklabels(mode_labels)
#         ax.grid()
#         ax.legend()

#         canvas = FigureCanvasTkAgg(fig, master=parent)
#         canvas.get_tk_widget().pack(fill="both", expand=True)
#         self.add_zoom_controls(fig, parent)
#         canvas.draw()

#     def create_servo_tab(self, parent):
#         """Servo analysis using pure numpy arrays."""
#         fig = plt.Figure(figsize=(10, 4))
#         ax = fig.add_subplot(1, 1, 1)
        
#         # Convert to numpy explicitly
#         mask = ~self.clean_df[['timestamp', 'servo pos']].isna().any(axis=1)
#         data = self.clean_df.loc[mask, ['timestamp', 'servo pos']]
#         t = data['timestamp'].to_numpy()
#         servo = data['servo pos'].to_numpy()

#         ax.plot(t, servo, 'purple')
#         ax.set_xlabel('Time (s)')
#         ax.set_ylabel('Servo Position')
#         ax.grid()

#         canvas = FigureCanvasTkAgg(fig, master=parent)
#         canvas.get_tk_widget().pack(fill="both", expand=True)
#         self.add_zoom_controls(fig, parent)
#         canvas.draw()
    
#     def add_zoom_controls(self, fig, parent):
#         """Add zoom buttons to a figure"""
#         zoom_frame = ttk.Frame(parent)
#         zoom_frame.pack(side="bottom", fill="x")
        
#         # Zoom buttons
#         ttk.Button(zoom_frame, text="Zoom In", 
#                 command=lambda: self.zoom(1.2, fig)).pack(side="left")
#         ttk.Button(zoom_frame, text="Zoom Out", 
#                 command=lambda: self.zoom(0.8, fig)).pack(side="left")
#         ttk.Button(zoom_frame, text="Reset", 
#                 command=lambda: self.reset_zoom(fig)).pack(side="left")
        
#         # Mouse wheel zoom
#         def on_scroll(event):
#             self.zoom(1.2 if event.button == 'up' else 0.8, fig)
#         fig.canvas.mpl_connect('scroll_event', on_scroll)

#     def zoom(self, factor, fig):
#         """Zoom all axes by a factor"""
#         for ax in fig.get_axes():
#             xlim = ax.get_xlim()
#             ylim = ax.get_ylim()
#             ax.set_xlim([xlim[0]*factor, xlim[1]*factor])
#             ax.set_ylim([ylim[0]*factor, ylim[1]*factor])
#         fig.canvas.draw_idle()

#     def reset_zoom(self, fig):
#         """Reset zoom to default"""
#         for ax in fig.get_axes():
#             ax.relim()
#             ax.autoscale_view()
#         fig.canvas.draw_idle()

# def main():
#     parser = argparse.ArgumentParser(description='PX4 Log Analyzer')
#     parser.add_argument('log_file', help='Path to CSV log file')
#     args = parser.parse_args()
    
#     if not os.path.exists(args.log_file):
#         print(f"Error: File not found - {args.log_file}")
#         return
    
#     DroneLogAnalyzer(args.log_file)

# if __name__ == "__main__":
#     main()

# =============================== =============================== =============================== ===============================

# import pandas as pd
# import matplotlib.pyplot as plt
# import plotly.express as px
# from datetime import datetime
# import os
# import argparse

# def load_and_preprocess(log_file):
#     """Load CSV log and preprocess data."""
#     df = pd.read_csv(log_file)
    
#     # Convert timestamp to relative seconds
#     df['timestamp'] -= df['timestamp'].min()
    
#     # Map PX4 nav_state to human-readable modes
#     px4_modes = {
#         0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
#         14: "OFFBOARD", 18: "AUTO_LAND", 20: "PRECLAND"
#     }
#     df['mode'] = df['mode'].map(px4_modes)
    
#     return df

# def plot_basic_trajectory(df, output_dir):
#     """2D/3D trajectory plots."""
#     plt.figure(figsize=(12, 6))
    
#     # 2D XY Plot
#     plt.subplot(1, 2, 1)
#     plt.plot(df['x'], df['y'], label='Actual Path')
#     plt.plot(df['traj_x'], df['traj_y'], 'r--', label='Setpoint')
#     plt.xlabel("X (North, m)"); plt.ylabel("Y (East, m)")
#     plt.title("XY Trajectory"); plt.legend(); plt.grid()
    
#     # Altitude Profile
#     plt.subplot(1, 2, 2)
#     plt.plot(df['timestamp'], df['z'], label='Actual')
#     plt.plot(df['timestamp'], df['traj_z'], 'r--', label='Setpoint')
#     plt.xlabel("Time (s)"); plt.ylabel("Z (Down, m)")
#     plt.title("Altitude Tracking"); plt.legend(); plt.grid()
    
#     plt.tight_layout()
#     plt.savefig(f"{output_dir}/trajectory_2d.png")
#     plt.close()

# def plot_interactive_3d(df, output_dir):
#     """Interactive 3D plot with Plotly."""
#     fig = px.line_3d(df, x='x', y='y', z='z', 
#                     color='mode', 
#                     hover_data=['timestamp', 'lidar_altitude', 'aruco id'],
#                     title="3D Flight Path with Mode Transitions")
#     fig.write_html(f"{output_dir}/trajectory_3d.html")

# def plot_control_performance(df, output_dir):
#     """Error analysis between setpoints and actuals."""
#     df['x_error'] = df['traj_x'] - df['x']
#     df['y_error'] = df['traj_y'] - df['y']
#     df['z_error'] = df['traj_z'] - df['z']
    
#     plt.figure(figsize=(12, 8))
#     plt.subplot(3, 1, 1)
#     plt.plot(df['timestamp'], df['x_error'], label='X Error')
#     plt.ylabel("Error (m)"); plt.title("Position Control Errors"); plt.grid()
    
#     plt.subplot(3, 1, 2)
#     plt.plot(df['timestamp'], df['y_error'], label='Y Error')
#     plt.ylabel("Error (m)"); plt.grid()
    
#     plt.subplot(3, 1, 3)
#     plt.plot(df['timestamp'], df['z_error'], label='Z Error')
#     plt.xlabel("Time (s)"); plt.ylabel("Error (m)"); plt.grid()
    
#     plt.tight_layout()
#     plt.savefig(f"{output_dir}/control_errors.png")
#     plt.close()

# def plot_mode_transitions(df, output_dir):
#     """Visualize when mode changes occurred."""
#     fig, ax = plt.subplots(figsize=(12, 4))
    
#     for mode in df['mode'].unique():
#         mode_data = df[df['mode'] == mode]
#         ax.scatter(mode_data['timestamp'], mode_data['mode'], label=mode)
    
#     ax.set_xlabel("Time (s)")
#     ax.set_title("PX4 Mode Transitions")
#     ax.legend(loc='upper right')
#     plt.savefig(f"{output_dir}/mode_transitions.png")
#     plt.close()

# def main():
#     parser = argparse.ArgumentParser(description='Drone Log Visualizer')
#     parser.add_argument('log_file', help='Path to CSV log file')
#     args = parser.parse_args()
    
#     # Create output directory
#     output_dir = f"analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
#     os.makedirs(output_dir, exist_ok=True)
    
#     # Process data
#     df = load_and_preprocess(args.log_file)
    
#     # Generate plots
#     plot_basic_trajectory(df, output_dir)
#     plot_control_performance(df, output_dir)
#     plot_mode_transitions(df, output_dir)
#     plot_interactive_3d(df, output_dir)
    
#     print(f"Analysis complete! Results saved to: {os.path.abspath(output_dir)}")

# if __name__ == "__main__":
#     main()

# =============================== =============================== =============================== ===============================

# import pandas as pd
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import numpy as np
# from datetime import datetime, timedelta

# # Load and prepare data
# df = pd.read_csv('logs/odometry_lidar_20250403_141206.csv')

# # Convert ALL relevant columns to numpy arrays upfront
# x = df['x'].to_numpy()
# y = df['y'].to_numpy()
# z = df['z'].to_numpy()
# lidar = df['lidar_altitude'].to_numpy()
# timestamps = df['timestamp'].to_numpy()  # Convert timestamps to numpy array

# # Coordinate conversions
# z_display = -z  # FRD to NED
# lidar_display = -lidar  # FRD to NED

# # Convert timestamps to datetime and calculate duration
# datetime_vals = pd.to_datetime(timestamps, unit='s')
# duration = datetime_vals[-1] - datetime_vals[0]
# mean_agl = np.mean(lidar_display)

# # Create plots
# fig = plt.figure(figsize=(18, 10))
# fig.suptitle(f"Flight Analysis | Duration: {duration} | Mean AGL: {mean_agl:.2f}m", y=1.02)

# # 1. 3D Trajectory
# ax1 = fig.add_subplot(231, projection='3d')
# ax1.plot(x, y, z_display, c='blue', alpha=0.5, linewidth=1)
# ax1.scatter(x[0], y[0], z_display[0], c='green', s=100, label='Start')
# ax1.scatter(x[-1], y[-1], z_display[-1], c='red', s=100, label='End')
# ax1.set_xlabel('X (m)')
# ax1.set_ylabel('Y (m)')
# ax1.set_zlabel('Altitude (m)')
# ax1.legend()

# # 2. XY Trajectory
# ax2 = fig.add_subplot(232)
# ax2.plot(x, y, 'b-', alpha=0.5)
# ax2.scatter(x[0], y[0], c='green', s=100, label='Start')
# ax2.scatter(x[-1], y[-1], c='red', s=100, label='End')
# ax2.set_xlabel('X (m)')
# ax2.set_ylabel('Y (m)')
# ax2.set_aspect('equal')
# ax2.grid(True)

# # 3. Altitude Profile (using numpy arrays for timestamps)
# ax3 = fig.add_subplot(233)
# ax3.plot(timestamps, z_display, label='Odometry')
# ax3.plot(timestamps, [mean_agl]*len(z_display), 'r--', label='Lidar AGL')
# ax3.set_xlabel('Timestamp (s)')
# ax3.set_ylabel('Altitude (m)')
# ax3.legend()

# # 4. Altitude Difference
# ax4 = fig.add_subplot(234)
# ground_level = z_display - lidar_display
# ax4.plot(timestamps, ground_level)
# ax4.set_xlabel('Timestamp (s)')
# ax4.set_ylabel('Ground Level (m)')
# ax4.set_title('Calculated Ground Elevation')

# # 5. Speed Analysis
# dx = np.diff(x)
# dy = np.diff(y)
# dz = np.diff(z_display)
# dt = np.diff(timestamps)
# speed = np.sqrt(dx**2 + dy**2 + dz**2)/dt

# ax5 = fig.add_subplot(235)
# ax5.plot(timestamps[1:], speed)
# ax5.set_xlabel('Timestamp (s)')
# ax5.set_ylabel('Speed (m/s)')
# ax5.set_title('Drone Speed')

# # 6. Lidar Health Check
# ax6 = fig.add_subplot(236)
# ax6.plot(timestamps, lidar_display, 'r-')
# ax6.set_xlabel('Timestamp (s)')
# ax6.set_ylabel('Lidar AGL (m)')
# ax6.set_title('Lidar Readings')

# plt.tight_layout()
# plt.show()

# =============================== =============================== =============================== ===============================

# import pandas as pd
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import numpy as np
# import os
# from matplotlib.collections import LineCollection
# from datetime import datetime, timedelta

# # Load the data
# log_path = os.path.join(os.path.expanduser('~'), 'zephyrDock', 'zephyrDock', 'logs', 'odometry_20250403_024114.csv')
# df = pd.read_csv(log_path)

# # 1. Reverse Z values (convert FRD to NED for visualization)
# df['z_display'] = -df['z']  # Flip the sign for visualization

# # 2. Calculate actual duration
# start_time = datetime.fromtimestamp(df['timestamp'].iloc[0])
# end_time = datetime.fromtimestamp(df['timestamp'].iloc[-1])
# duration = end_time - start_time
# duration_str = str(timedelta(seconds=duration.total_seconds())).split(".")[0]  # Remove microseconds

# # Create normalized time values (0-1) for coloring
# normalized_time = np.linspace(0, 1, len(df))

# # Convert to numpy arrays
# x = df['x'].values
# y = df['y'].values
# z_display = df['z_display'].values
# timestamps = df['timestamp'].values

# # Create figure with larger size
# plt.figure(figsize=(14, 6))

# # --- 2D XY Plot with Color Gradient ---
# ax1 = plt.subplot(131)
# points = np.array([x, y]).T.reshape(-1, 1, 2)
# segments = np.concatenate([points[:-1], points[1:]], axis=1)

# lc = LineCollection(segments, cmap='viridis', norm=plt.Normalize(0, 1))
# lc.set_array(normalized_time)
# lc.set_linewidth(2)
# line = ax1.add_collection(lc)

# # Add start/end markers
# ax1.scatter(x[0], y[0], c='green', s=100, label=f'Start\n{datetime.fromtimestamp(timestamps[0]).strftime("%H:%M:%S")}', zorder=3)
# ax1.scatter(x[-1], y[-1], c='red', s=100, label=f'End\n{datetime.fromtimestamp(timestamps[-1]).strftime("%H:%M:%S")}', zorder=3)

# ax1.set_xlabel('X Position (m)')
# ax1.set_ylabel('Y Position (m)')
# ax1.set_title(f'XY Trajectory\nTotal Duration: {duration_str}')
# ax1.grid(True)
# ax1.legend()
# plt.colorbar(line, ax=ax1, label='Progress')

# # --- 3D Plot with Color Gradient ---
# ax2 = plt.subplot(132, projection='3d')
# for i in range(len(x)-1):
#     ax2.plot(x[i:i+2], y[i:i+2], z_display[i:i+2], 
#              color=plt.cm.viridis(normalized_time[i]),
#              alpha=0.8, linewidth=2)

# # Add start/end markers
# ax2.scatter(x[0], y[0], z_display[0], c='green', s=100, label='Start')
# ax2.scatter(x[-1], y[-1], z_display[-1], c='red', s=100, label='End')

# ax2.set_xlabel('X (m)')
# ax2.set_ylabel('Y (m)')
# ax2.set_zlabel('Altitude (m)')  # Changed from Z to Altitude
# ax2.set_title('3D Flight Path')

# # --- Altitude Profile with Correct Time ---
# ax3 = plt.subplot(133)
# relative_time = timestamps - timestamps[0]  # Seconds since start

# for i in range(len(z_display)-1):
#     ax3.plot([relative_time[i], relative_time[i+1]], 
#              [z_display[i], z_display[i+1]], 
#              color=plt.cm.viridis(normalized_time[i]),
#              linewidth=2)

# ax3.scatter(relative_time[0], z_display[0], c='green', s=100, label='Start')
# ax3.scatter(relative_time[-1], z_display[-1], c='red', s=100, label='End')

# ax3.set_xlabel(f'Time (seconds)\nStart: {datetime.fromtimestamp(timestamps[0]).strftime("%H:%M:%S")}')
# ax3.set_ylabel('Altitude (m)')
# ax3.set_title('Altitude Profile Over Time')
# ax3.grid(True)
# ax3.legend()

# # Add colorbar for altitude plot
# sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(0, max(relative_time)))
# sm.set_array([])
# cbar = plt.colorbar(sm, ax=ax3)
# cbar.set_label('Time (seconds)')

# plt.tight_layout()
# plt.show()

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