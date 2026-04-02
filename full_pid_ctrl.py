import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt

class CarterPID(Node):
    def __init__(self):
        super().__init__('carter_pid')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # ---- TARGET ----
        self.target_x = 0.2
        self.target_y = 1
        
        # ---- PID GAINS ----
        self.kp_v = 0.5;  self.ki_v = 0.05; self.kd_v = 0.2
        self.kp_w = 1.2;  self.ki_w = 0.01; self.kd_w = 0.3

        # ---- STATE MEMORY ----
        self.prev_time = 0.0
        self.start_time = 0.0
        self.integral_v = 0.0; self.prev_error_v = 0.0
        self.integral_w = 0.0; self.prev_error_w = 0.0
        self.target_reached = False

        # ---- DATA LOGGING ARRAYS FOR PLOTTING ----
        self.history_t = []
        self.history_x = []
        self.history_y = []
        self.history_ev = []
        self.history_ew = []
        self.history_v = []
        self.history_w = []

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg):
        if self.target_reached:
            return  # Stop calculating if we are already there

        # 1. TIME
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        current_time = sec + (nanosec * 1e-9)

        if self.prev_time == 0.0:
            self.prev_time = current_time
            self.start_time = current_time
            return
            
        dt = current_time - self.prev_time
        if dt <= 0.0: return
        
        elapsed_time = current_time - self.start_time

        # 2. READ STATE
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        print(f"current X:", {current_x})
        print(f"current Y:", {current_y})
        q = msg.pose.pose.orientation
        current_theta = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        # 3. ERRORS
        error_x = self.target_x - current_x
        error_y = self.target_y - current_y
        error_v = math.hypot(error_x, error_y)
        
        target_angle = math.atan2(error_y, error_x)
        error_w = target_angle - current_theta
        error_w = math.atan2(math.sin(error_w), math.cos(error_w)) # Normalize [-pi, pi]

        # 4. PID MATH
        self.integral_v += error_v * dt
        self.integral_w += error_w * dt
        self.integral_v = max(min(self.integral_v, 2.0), -2.0) # Clamp between -2 and 2
        self.integral_w = max(min(self.integral_w, 2.0), -2.0)
        # self.integral_v = max(min(self.integral_v + (error_v * dt), 2.0), -2.0)
        # self.integral_w = max(min(self.integral_w + (error_w * dt), 2.0), -2.0)
        
        derivative_v = (error_v - self.prev_error_v) / dt
        derivative_w = (error_w - self.prev_error_w) / dt
        
        v = (self.kp_v * error_v) + (self.ki_v * self.integral_v) + (self.kd_v * derivative_v)
        w = (self.kp_w * error_w) + (self.ki_w * self.integral_w) + (self.kd_w * derivative_w)

        if abs(error_w) > 0.5: 
            v = v * 0.1  # Turn before driving

        v = max(min(v, 1.5), -1.5) 
        w = max(min(w, 2.0), -2.0)

        # 5. LOG DATA FOR PLOTTING
        self.history_t.append(elapsed_time)
        self.history_x.append(current_x)
        self.history_y.append(current_y)
        self.history_ev.append(error_v)
        self.history_ew.append(error_w)
        self.history_v.append(v)
        self.history_w.append(w)

        # 6. STOP CONDITION
        if error_v < 0.02:
            v = 0.0
            w = 0.0
            self.get_logger().info("Target Reached! Close the plot window to exit.")
            self.target_reached = True

        # 7. PUBLISH COMMAND
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

        self.prev_time = current_time
        self.prev_error_v = error_v
        self.prev_error_w = error_w

    def plot_graphs(self):
        """Generates control engineering plots from the logged data."""
        if not self.history_t:
            print("No data logged.")
            return

        plt.figure(figsize=(12, 8))
        plt.suptitle('Carter Robot - PID Control Performance', fontsize=16)

        # --- Plot 1: 2D Trajectory (Top Left) ---
        plt.subplot(2, 2, 1)
        plt.plot(self.history_x, self.history_y, label='Robot Path', color='blue', linewidth=2)
        plt.scatter([self.history_x[0]], [self.history_y[0]], color='green', marker='o', s=100, label='Start')
        plt.scatter([self.target_x], [self.target_y], color='red', marker='X', s=100, label='Target')
        plt.title('X-Y Trajectory')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.grid(True); plt.legend(); plt.axis('equal')

        # --- Plot 2: Diminishing Errors (Top Right) ---
        plt.subplot(2, 2, 2)
        plt.plot(self.history_t, self.history_ev, label='Distance Error (m)', color='red')
        plt.plot(self.history_t, self.history_ew, label='Heading Error (rad)', color='orange')
        plt.axhline(0, color='black', linewidth=1, linestyle='--')
        plt.title('Error vs Time (Step Response)')
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.grid(True); plt.legend()

        # --- Plot 3: Control Effort (Bottom spanning both columns) ---
        plt.subplot(2, 1, 2)
        plt.plot(self.history_t, self.history_v, label='Cmd Lin Vel ($v$) [m/s]', color='purple')
        plt.plot(self.history_t, self.history_w, label='Cmd Ang Vel ($\omega$) [rad/s]', color='cyan')
        plt.axhline(0, color='black', linewidth=1, linestyle='--')
        plt.title('Control Action (Actuator Effort)')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity Command')
        plt.grid(True); plt.legend()

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = CarterPID()
    print("PID Controller Started. Waiting to reach target or press Ctrl+C to plot...")
    
    try:
        # Spin keeps the node running. We check internally if target is reached.
        while rclpy.ok() and not node.target_reached:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print("\nRun interrupted by user.")
    finally:
        # Whether it finished or you pressed Ctrl+C, plot the data!
        print("Generating plots...")
        
        # Stop the robot immediately before plotting
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        
        node.plot_graphs()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
