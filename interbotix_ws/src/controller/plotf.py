#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re

class ForceMonitorPlotter(Node):
    def __init__(self):
        super().__init__('force_monitor_plotter')
        
        # Configuration
        self.topic_name = '/force_feedback/joint_monitor'
        self.max_points = 200  # Number of points to display
        
        # Data storage
        self.times = deque(maxlen=self.max_points)
        self.force_x = deque(maxlen=self.max_points)
        self.force_y = deque(maxlen=self.max_points)
        self.force_z = deque(maxlen=self.max_points)
        self.start_time = None
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            self.topic_name,
            self.callback,
            10
        )
        
        # Setup plot with 3 subplots
        self.fig, self.axes = plt.subplots(3, 1, figsize=(10, 8))
        self.lines = []
        
        labels = ['Force X', 'Force Y', 'Force Z']
        colors = ['r', 'g', 'b']
        
        for i, (ax, label, color) in enumerate(zip(self.axes, labels, colors)):
            line, = ax.plot([], [], color=color, linewidth=2)
            self.lines.append(line)
            ax.set_ylabel(f'{label} (N)')
            ax.set_title(f'{label} Component')
            ax.grid(True, alpha=0.3)
            if i == 2:
                ax.set_xlabel('Time (s)')
        
        self.fig.suptitle('Force Feedback Monitor - Real-time Plot', fontsize=14, fontweight='bold')
        self.fig.tight_layout()
        
        self.get_logger().info(f'Plotting force data from topic: {self.topic_name}')
    
    def callback(self, msg):
        # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.start_time is None:
            self.start_time = current_time
        
        relative_time = current_time - self.start_time
        
        # Parse force values from the message
        # Expected format: F:[ 0.000, 0.000, 0.000]N
        pattern = r'F:\[\s*(-?\d+\.?\d*),\s*(-?\d+\.?\d*),\s*(-?\d+\.?\d*)\]N'
        match = re.search(pattern, msg.data)
        
        if match:
            fx = float(match.group(1))
            fy = float(match.group(2))
            fz = float(match.group(3))
            
            # Store data
            self.times.append(relative_time)
            self.force_x.append(fx)
            self.force_y.append(fy)
            self.force_z.append(fz)
            
            self.get_logger().debug(f'F:[{fx:.3f}, {fy:.3f}, {fz:.3f}]N at t={relative_time:.2f}s')
        else:
            self.get_logger().warning(f'Could not parse force data from: {msg.data}')
    
    def update_plot(self, frame):
        if len(self.times) > 0:
            time_list = list(self.times)
            force_data = [list(self.force_x), list(self.force_y), list(self.force_z)]
            
            for line, data, ax in zip(self.lines, force_data, self.axes):
                line.set_data(time_list, data)
                ax.relim()
                ax.autoscale_view()
        
        return self.lines
    
    def run(self):
        # Create animation
        ani = animation.FuncAnimation(
            self.fig, 
            self.update_plot, 
            interval=50,  # Update every 50ms
            blit=True
        )
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter = ForceMonitorPlotter()
    
    # Spin in a separate thread to allow matplotlib to run
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(plotter,))
    spin_thread.daemon = True
    spin_thread.start()
    
    try:
        plotter.run()
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
