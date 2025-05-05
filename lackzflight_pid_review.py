#!/usr/bin/env python3
# # DISCLAIMER # #

# USED TO REVIEW CUSTOM PRECISION DESCENT MODE PID PERFORMANCE #
# OUTDATED BUT SUITABLE FOR FILE THAT IS NOT READABLE VIA LATEST VER. #
# LOG NAME IS pid_log / pid_sim_log IN /logs #







import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from tkinter import Tk, filedialog
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
import os

class PIDAnalyzer:
    def __init__(self, root):
        self.root = root
        self.root.title("PID Performance Analyzer")
        
        # Create a dedicated filename display bar at top
        self.filename_frame = tk.Frame(root, bg='#e6e6e6', height=30, bd=1, relief=tk.SUNKEN)
        self.filename_frame.pack(fill=tk.X, side=tk.TOP, padx=5, pady=5)
        
        # Filename label with large, bold text
        self.filename_label = tk.Label(
            self.filename_frame,
            text="No file loaded",
            bg='#e6e6e6',
            fg='black',
            font=('TkDefaultFont', 10, 'bold'),
            anchor=tk.W,
            padx=10
        )
        self.filename_label.pack(fill=tk.X, side=tk.LEFT)
        
        # Main content area
        self.main_frame = ttk.Frame(root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create tabs
        self.tab_control = ttk.Notebook(self.main_frame)
        self.plot_tab = ttk.Frame(self.tab_control)
        self.metrics_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.plot_tab, text='Plots')
        self.tab_control.add(self.metrics_tab, text='Metrics')
        self.tab_control.pack(expand=1, fill="both")
        
        # Plot tab content
        self.fig = plt.Figure(figsize=(12, 8), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_tab)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Metrics tab content
        self.metrics_text = tk.Text(self.metrics_tab, wrap=tk.WORD)
        self.metrics_text.pack(fill=tk.BOTH, expand=True)
        
        # Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Ready")
        self.status_bar = tk.Label(
            root,
            textvariable=self.status_var,
            bd=1,
            relief=tk.SUNKEN,
            anchor=tk.W,
            bg='#f0f0f0',
            padx=10
        )
        self.status_bar.pack(fill=tk.X, side=tk.BOTTOM)
        
        # Menu
        self.menubar = tk.Menu(root)
        self.filemenu = tk.Menu(self.menubar, tearoff=0)
        self.filemenu.add_command(label="Open CSV", command=self.load_file)
        self.filemenu.add_command(label="Exit", command=root.quit)
        self.menubar.add_cascade(label="File", menu=self.filemenu)
        root.config(menu=self.menubar)

    def load_file(self):
        file_path = filedialog.askopenfilename(
            title="Select PID Log CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if file_path:
            self.current_file = file_path
            short_name = os.path.basename(file_path)
            
            # Immediately update UI
            self.filename_label.config(text=f"File: {short_name}", fg='blue')
            self.status_var.set(f"Processing {short_name}...")
            self.root.update()  # Force UI refresh
            
            try:
                self.analyze_data(file_path)
                self.filename_label.config(fg='green')
                self.status_var.set("Analysis complete")
            except Exception as e:
                self.filename_label.config(text=f"Error: {short_name}", fg='red')
                self.status_var.set(f"Error: {str(e)}")
                self.metrics_text.insert(tk.END, f"\nError occurred:\n{str(e)}\n")

    def analyze_data(self, csv_file):
        # Load and prepare data
        df = pd.read_csv(csv_file)
        time_vals = np.arange(len(df)) * 0.01  # 100Hz assumption
        df['time_elapsed'] = time_vals
        
        # Clear previous plots
        self.fig.clf()
        
        # Create subplots
        gs = self.fig.add_gridspec(2, 2, wspace=0.3, hspace=0.4)
        ax1 = self.fig.add_subplot(gs[0, 0])
        ax2 = self.fig.add_subplot(gs[0, 1])
        ax3 = self.fig.add_subplot(gs[1, 0])
        ax4 = self.fig.add_subplot(gs[1, 1])
        
        # Plot data
        self.plot_axis_data(ax1, df, 'X')
        self.plot_axis_data(ax2, df, 'Y')
        self.plot_position_data(ax3, df, 'X')
        self.plot_position_data(ax4, df, 'Y')
        
        # Calculate metrics
        metrics = self.calculate_metrics(df)
        
        # Update display
        self.canvas.draw()
        self.update_metrics(metrics)
        
        # Save plot
        output_file = os.path.splitext(csv_file)[0] + '_analysis.png'
        self.fig.savefig(output_file, dpi=300, bbox_inches='tight')

    def plot_axis_data(self, ax, df, axis):
        time = df['time_elapsed'].to_numpy()
        error = df[f'error_{axis.lower()}'].to_numpy()
        vel = df[f'v{axis.lower()}'].to_numpy()
        
        color = 'r' if axis == 'X' else 'g'
        
        ax.plot(time, error, f'{color}-', label=f'{axis} Error')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (m)', color=color)
        ax.tick_params(axis='y', labelcolor=color)
        ax.grid(True)
        
        ax2 = ax.twinx()
        ax2.plot(time, vel, 'b-', label=f'V{axis} Cmd')
        ax2.set_ylabel('Velocity (m/s)', color='b')
        ax2.tick_params(axis='y', labelcolor='b')
        ax.set_title(f'{axis}-Axis Performance')

    def plot_position_data(self, ax, df, axis):
        time = df['time_elapsed'].to_numpy()
        drone_pos = df[f'drone_{axis.lower()}'].to_numpy()
        tag_pos = df[f'tag_{axis.lower()}'].to_numpy()
        
        ax.plot(time, drone_pos, 'b-', label=f'Drone {axis}')
        ax.plot(time, tag_pos, 'r--', label=f'Tag {axis}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        ax.legend()
        ax.grid(True)
        ax.set_title(f'{axis} Position Tracking')

    def calculate_metrics(self, df):
        metrics = {}
        for axis in ['x', 'y']:
            error = df[f'error_{axis}'].to_numpy()
            metrics[axis] = {
                'final_error': error[-1],
                'max_error': np.max(np.abs(error)),
                'rms_error': np.sqrt(np.mean(error**2)),
                'settling_index': np.argmax(np.abs(error) < 0.05) / len(error) if any(np.abs(error) < 0.05) else None
            }
        return metrics

    def update_metrics(self, metrics):
        self.metrics_text.delete(1.0, tk.END)
        
        text = "=== Performance Metrics ===\n\n"
        for axis in ['x', 'y']:
            text += f"{axis.upper()}-Axis:\n"
            text += f"  Final Error: {metrics[axis]['final_error']:.4f} m\n"
            text += f"  Max Error: {metrics[axis]['max_error']:.4f} m\n"
            text += f"  RMS Error: {metrics[axis]['rms_error']:.4f} m\n"
            if metrics[axis]['settling_index']:
                text += f"  Settling Time: {metrics[axis]['settling_index']*100:.1f}% of trajectory\n"
            text += "\n"
        
        self.metrics_text.insert(tk.END, text)

if __name__ == "__main__":
    root = tk.Tk()
    app = PIDAnalyzer(root)
    root.geometry("1000x800")
    root.mainloop()