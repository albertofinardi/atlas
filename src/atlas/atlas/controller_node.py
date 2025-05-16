import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import sys


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_controller')
        
        self.angular_velocity = 0.0
        self.linear_velocity = 0.0
        self.stop_line_detected = False
        
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.steering_subscriber = self.create_subscription(
            Float32, 'line_pid/steering', self.steering_callback, 10)
        
        self.stop_line_subscriber = self.create_subscription(
            Bool, 'line_pid/stop_line_detected', self.stop_line_callback, 10)
        
        # Controller parameters for high-speed operation
        self.declare_parameter('linear_velocity', 0.25)
        self.declare_parameter('no_line_timeout', 1.0) 
        self.declare_parameter('startup_delay', 5.0)

        self.linear_velocity = self.get_parameter('linear_velocity').value
        
        self.update_timer = None
        self.startup_timer = None

        self.last_control_update_time = None
        
        self.is_stopped = False
        
        self.get_logger().info('Controller Node initialized')
    
    def start(self):
        # First stop the robot
        self.stop()
        startup_delay = self.get_parameter('startup_delay').value
        
        # Create a one-shot timer for the startup delay
        self.get_logger().info(f'Waiting {startup_delay} seconds before starting...')
        self.startup_timer = self.create_timer(startup_delay, self.startup_complete_callback)
        
    def startup_complete_callback(self):
        # Cancel the startup timer since it's a one-time event
        self.startup_timer.cancel()
        
        # Create the regular update timer
        self.update_timer = self.create_timer(1/30, self.update_callback)
        self.is_stopped = False
        self.get_logger().info("Controller Node active.")
    
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
        self.is_stopped = True
    
    def steering_callback(self, msg):
        self.angular_velocity = msg.data * 2
        self.last_control_update_time = self.get_clock().now()
    
    def stop_line_callback(self, msg):
        self.stop_line_detected = msg.data
        
        if self.stop_line_detected:
            self.stop() # Stop the robot if a stop line is detected
    
    def check_timeout(self):
        """Check if we've lost control updates for too long"""
        if self.last_control_update_time is None:
            return True  # No control update received yet
            
        current_time = self.get_clock().now()
        timeout_duration = self.get_parameter('no_line_timeout').value
        
        elapsed = (current_time - self.last_control_update_time).nanoseconds / 1e9
        return elapsed > timeout_duration
    
    def update_callback(self):
        if self.is_stopped:
            return
        
        cmd_vel = Twist()
        
        if self.check_timeout():
            self.get_logger().warn('No control updates for too long, stopping robot')
            self.stop()
        else:
            cmd_vel.linear.x = self.linear_velocity
            cmd_vel.angular.z = self.angular_velocity
            
            self.get_logger().debug(
                f'Control: linear={cmd_vel.linear.x:.2f}, angular={cmd_vel.angular.z:.2f}'
            )
            
            self.vel_publisher.publish(cmd_vel)


def main():
    rclpy.init(args=sys.argv)
    
    node = ControllerNode()
    node.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()