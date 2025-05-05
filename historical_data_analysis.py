#!/usr/bin/env python3
# # DISCLAIMER # #

# USED TO REVIEW FLIGHT TRAJECTORY AND STATUS CHANGED #
# LOG NAME IS zd_logging / zd_sim_logging IN /logs #
# THERE'RE MULTIPLE VERSION BUT THIS IS THE LATEST FOR NOW # 
# ADJUST AS U PLEASE #








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
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 4: "HOLD",
    14: "OFFBOARD", 18: "AUTO_LAND",  23:"CUSTOM_PRECISION_LAND",
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
        status_frame.pack(fill='x', side='bottom', pady=(5, 0))

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
                padding=(5, 2),
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
            ttk.Label(status_frame, text="No file loaded", style='Status.TLabel').pack()

        # Notebook for tabs - placed above status bar
        notebook = ttk.Notebook(main_frame)
        notebook.pack(fill='both', expand=True)

        # Rest of your tab creation code...
        tabs = [
            ("XY Trajectory", self.create_xy_trajectory_tab),
            ("Altitude", self.create_altitude_tab),
            ("Errors", self.create_errors_tab),
            ("Modes", self.create_modes_tab),
            ("Velocity", self.create_velocity_tab),  # Add the new velocity tab
        ]

        for name, func in tabs:
            tab = ttk.Frame(notebook)
            notebook.add(tab, text=name)
            try:
                func(tab)
            except Exception as e:
                ttk.Label(tab, text=f"Error: {str(e)}").pack()

        # Force UI update
        self.root.update_idletasks()


    def create_velocity_tab(self, parent):
        """Velocity tracking for x, y, z, and yaw."""
        fig = plt.Figure(figsize=(10, 8))
        
        # Convert to numpy explicitly
        mask = ~self.clean_df[['timestamp', 'velo_x', 'velo_y', 'velo_z', 'velo_yaw']].isna().any(axis=1)
        data = self.clean_df.loc[mask, ['timestamp', 'velo_x', 'velo_y', 'velo_z', 'velo_yaw']]
        t = data['timestamp'].to_numpy()
        velo_x = data['velo_x'].to_numpy()
        velo_y = data['velo_y'].to_numpy()
        velo_z = data['velo_z'].to_numpy()
        velo_yaw = data['velo_yaw'].to_numpy()

        # Velocity plot for X, Y, Z
        ax1 = fig.add_subplot(2, 1, 1)
        ax1.plot(t, velo_x, label='Velo X', color='r')
        ax1.plot(t, velo_y, label='Velo Y', color='b')
        ax1.plot(t, velo_z, label='Velo Z', color='g')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.legend()
        ax1.grid()

        # Velocity plot for Yaw
        ax2 = fig.add_subplot(2, 1, 2)
        ax2.plot(t, velo_yaw, label='Velo Yaw', color='orange')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Yaw Velocity (rad/s)')
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
        ax1.plot(t, z_sp, '--', label='Setpoint')
        ax1.plot(t, z, label='Actual')
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
        mode_keys = list(PX4_MODE_MAP.keys())
        mode_labels = list(PX4_MODE_MAP.values())
        for mode in data['mode_name'].unique():
            if mode in mode_labels:
                mask = data['mode_name'] == mode
                ax.scatter(data.loc[mask, 'timestamp'].to_numpy(),
                          data.loc[mask, 'mode_num'].to_numpy(),
                          label=mode, s=10)
        
        ax.set_xlabel('Time (s)')
        ax.set_yticks(mode_keys)
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
                    content = event.ydata
                    time = event.xdata
                    label = simpledialog.askstring("Phase Marker", 
                                                f"Value: {content:.2f} Time: {time:.2f}s\nEnter phase name:",
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