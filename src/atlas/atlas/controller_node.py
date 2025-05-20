import sys
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String


class TrafficSign(Enum):
    """Enumeration of traffic signs and lights."""
    SPEED_30 = "SPEED_30"
    SPEED_50 = "SPEED_50"
    LIGHT_RED = "LIGHT_RED"
    LIGHT_YELLOW = "LIGHT_YELLOW"
    LIGHT_GREEN = "LIGHT_GREEN"


class ControllerNode(Node):
    """
    ROS2 node that controls vehicle movement based on various sensor inputs.
    
    Subscribes to:
    - Line detection (steering, stop lines, lost line)
    - Traffic sign/light detection
    
    Publishes:
    - Command velocity
    """
    
    def __init__(self):
        """Initialize the controller node and set up publishers/subscribers."""
        super().__init__('vehicle_controller')
        
        self._traffic_light_state = {
            "red_detected": False,
            "yellow_detected": False
        }
        self._stop_line_detected = False
        self._line_is_lost = False

        self._angular_velocity = 0.0
        self._current_linear_velocity = 0.0
        
        self._vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self._create_subscribers()
        
        self._declare_parameters()
        
        self._green_linear_velocity = self._linear_velocity_50
        
        self._startup_timer = None
        self._update_timer = None
        
        self.get_logger().info('Controller Node initialized')
    
    def _create_subscribers(self):
        """Set up subscription handlers."""
        self.create_subscription(
            Float32, 'line_pid/steering', self._steering_callback, 10)
        
        self.create_subscription(
            Bool, 'line_pid/stop_line_detected', self._stop_line_callback, 10)
        
        self.create_subscription(
            Bool, 'line_pid/lost_line_detected', self._lost_line_callback, 10)
        
        self.create_subscription(
            String, 'traffic/id', self._traffic_detection_callback, 10)
    
    def _declare_parameters(self):
        """Declare and get node parameters."""

        self.declare_parameter('linear_velocity_50', 1.0)
        self.declare_parameter('linear_velocity_30', 0.5)
        self.declare_parameter('angular_velocity_max', 2.0)
        self.declare_parameter('startup_delay', 5.0)
        
        self._linear_velocity_50 = self.get_parameter('linear_velocity_50').value
        self._linear_velocity_30 = self.get_parameter('linear_velocity_30').value
        self._angular_velocity_max = self.get_parameter('angular_velocity_max').value
        self._startup_delay = self.get_parameter('startup_delay').value
        
        # Set initial velocity
        self._current_linear_velocity = self._linear_velocity_50
    
    def start(self):
        """Start the controller after delay."""
        self.get_logger().info(f'Waiting {self._startup_delay} seconds before starting...')
        self._startup_timer = self.create_timer(self._startup_delay, self._startup_complete_callback)
    
    def _startup_complete_callback(self):
        """Called when startup delay is complete."""
        self._startup_timer.cancel()
        
        self._update_timer = self.create_timer(1/30, self._update_callback)
        self._is_stopped = False
        self.get_logger().info("Controller Node active")
    
    def stop(self):
        """Stop the vehicle."""
        self._current_linear_velocity = 0.0
        self._angular_velocity = 0.0
        self._publish_zero_velocity()
        self._is_stopped = True
        self.get_logger().info("Vehicle stopped")
    
    def _publish_zero_velocity(self):
        """Publish a zero velocity command."""
        cmd_vel = Twist()
        self._vel_publisher.publish(cmd_vel)
    
    def _steering_callback(self, msg):
        """Process steering commands from the PID controller."""
        if self._stop_line_detected:
            return
        
        self._angular_velocity = msg.data * self._angular_velocity_max
    
    def _stop_line_callback(self, msg):
        """Handle stop line detection."""
        self._stop_line_detected = msg.data
        
        if not self._stop_line_detected:
            return
            
        # Stop line handling logic
        if self._traffic_light_state["red_detected"]:
            self.stop()  # Stop the robot if a stop line is detected with red light
        elif self._traffic_light_state["yellow_detected"]:
            self._current_linear_velocity = self._green_linear_velocity # Go back to green velocity if stop line is detected with yellow light (passed the line)
    
    def _lost_line_callback(self, msg):
        """Handle lost line detection."""
        self._line_is_lost = msg.data
        
        if self._line_is_lost:
            self.stop()
    
    def _traffic_detection_callback(self, msg):
        """Process traffic sign/light detection."""
        sign = msg.data
        
        # Reset traffic light state
        self._traffic_light_state["red_detected"] = False
        self._traffic_light_state["yellow_detected"] = False
        
        # Handle different traffic signs
        if sign == TrafficSign.SPEED_30.value:
            self._current_linear_velocity = self._linear_velocity_30
            self._green_linear_velocity = self._current_linear_velocity
            self.get_logger().info("Speed limit 30 detected")
            
        elif sign == TrafficSign.SPEED_50.value:
            self._current_linear_velocity = self._linear_velocity_50
            self._green_linear_velocity = self._current_linear_velocity
            self.get_logger().info("Speed limit 50 detected")
            
        elif sign == TrafficSign.LIGHT_RED.value:
            self._current_linear_velocity = self._linear_velocity_30
            self._traffic_light_state["red_detected"] = True
            self.get_logger().info("Red light detected")
            
        elif sign == TrafficSign.LIGHT_YELLOW.value:
            self._current_linear_velocity = self._linear_velocity_30
            self._traffic_light_state["yellow_detected"] = True
            self.get_logger().info("Yellow light detected")
            
        elif sign == TrafficSign.LIGHT_GREEN.value:
            self._current_linear_velocity = self._green_linear_velocity
            self.get_logger().info("Green light detected")
    
    def _update_callback(self):
        """
        Main control loop.
        Publishes velocity commands based on current state.
        """
        if self._line_is_lost:
            self.stop()
            return
        
        cmd_vel = Twist()
        cmd_vel.linear.x = self._current_linear_velocity
        cmd_vel.angular.z = self._angular_velocity
        
        self.get_logger().debug(
            f'Control: linear={cmd_vel.linear.x:.2f}, angular={cmd_vel.angular.z:.2f}'
        )
        
        self._vel_publisher.publish(cmd_vel)


def main():
    rclpy.init(args=sys.argv)
    node = ControllerNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.stop()


if __name__ == '__main__':
    main()