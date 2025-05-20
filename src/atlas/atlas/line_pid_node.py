import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque


class CameraPosition:
    """Camera position constants."""
    LEFT = 'left'
    CENTER = 'center'
    RIGHT = 'right'


class LinePIDNode(Node):
    """
    ROS2 node that detects lines from camera images and computes steering commands.
    
    The node:
    - Subscribes to three camera feeds (left, center, right)
    - Processes images to detect line positions
    - Uses PID control to calculate appropriate steering
    - Detects stop lines and missing lines
    - Publishes steering commands and line detection results
    """
    
    def __init__(self):
        """Initialize the line detection and PID controller node."""
        super().__init__('line_detection_node')
        
        self._bridge = CvBridge()
        self._setup_publishers()
        self._setup_subscribers()
        self._declare_parameters()
        self._initialize_state()
        
        self._publish_timer = self.create_timer(1/30, self._publish_control)
        self._setup_debug_mode()
        self.get_logger().info('Line Detection Node initialized with normalized steering output (-1.0 to +1.0)')
    
    def _setup_publishers(self):
        """Set up all publishers."""
        # Publisher for normalized steering (-1.0 to +1.0)
        self._steering_pub = self.create_publisher(
            Float32, 'line_pid/steering', 10)
            
        # Publisher for stop line detection
        self._stop_line_pub = self.create_publisher(
            Bool, 'line_pid/stop_line_detected', 10)
        
        # Publisher for lost line detection
        self._lost_line_pub = self.create_publisher(
            Bool, 'line_pid/lost_line_detected', 10)
    
    def _setup_subscribers(self):
        """Set up camera subscribers."""
        self.create_subscription(
            Image, '/vision/left', self._left_camera_callback, 10)
        self.create_subscription(
            Image, '/vision/center', self._center_camera_callback, 10)
        self.create_subscription(
            Image, '/vision/right', self._right_camera_callback, 10)
    
    def _declare_parameters(self):
        """Declare parameters used by the node."""
        # Image processing parameters
        self.declare_parameter('threshold_value', 128)
        self.declare_parameter('min_contour_area', 3)
        self.declare_parameter('debug', False)
        
        # Stop line detection parameters
        self.declare_parameter('stop_line_max_deviation', 0.15)
        self.declare_parameter('stop_line_center_min', 0.4)
        self.declare_parameter('stop_line_center_max', 0.6)
        
        # PID parameters for each camera
        for camera in [CameraPosition.LEFT, CameraPosition.RIGHT]:
            self.declare_parameter(f'{camera}_kp', 0.8)
            self.declare_parameter(f'{camera}_ki', 0.02)
            self.declare_parameter(f'{camera}_kd', 0.35)

        self.declare_parameter('center_kp', 1.0)
        self.declare_parameter('center_ki', 0.02)
        self.declare_parameter('center_kd', 0.4)
        
        # Additional PID and control parameters
        self.declare_parameter('integral_windup_limit', 0.2)
        self.declare_parameter('corner_detection_threshold', 0.25)
    
    def _initialize_state(self):
        """Initialize all state variables."""
        # Line position state (0.0 = left edge, 0.5 = center, 1.0 = right edge)
        self._positions = {
            CameraPosition.LEFT: 0.5,
            CameraPosition.CENTER: 0.5,
            CameraPosition.RIGHT: 0.5
        }
        
        # Line detection state
        self._line_detected = {
            CameraPosition.LEFT: False,
            CameraPosition.CENTER: False,
            CameraPosition.RIGHT: False
        }
        
        # PID state variables
        self._integral = {
            CameraPosition.LEFT: 0.0,
            CameraPosition.CENTER: 0.0,
            CameraPosition.RIGHT: 0.0
        }
        
        self._previous_error = {
            CameraPosition.LEFT: 0.0,
            CameraPosition.CENTER: 0.0,
            CameraPosition.RIGHT: 0.0
        }
        
        # Time tracking
        self._previous_time = self.get_clock().now()
        
        # Derivative filtering
        self._derivative_filter_size = 3
        self._derivative_history = {
            CameraPosition.LEFT: deque([0.0] * self._derivative_filter_size, maxlen=self._derivative_filter_size),
            CameraPosition.CENTER: deque([0.0] * self._derivative_filter_size, maxlen=self._derivative_filter_size),
            CameraPosition.RIGHT: deque([0.0] * self._derivative_filter_size, maxlen=self._derivative_filter_size)
        }
        
        self._no_line_start_time = None
    
    def _setup_debug_mode(self):
        """Set up debug mode if enabled in parameters."""
        self._debug_images = self.get_parameter('debug').value
        
        if self._debug_images:
            self._debug_publishers = {
                CameraPosition.LEFT: self.create_publisher(Image, 'line_pid/debug/left_cam', 10),
                CameraPosition.CENTER: self.create_publisher(Image, 'line_pid/debug/center_cam', 10),
                CameraPosition.RIGHT: self.create_publisher(Image, 'line_pid/debug/right_cam', 10)
            }
            self.get_logger().info('Debug visualization enabled')
    
    def _left_camera_callback(self, msg):
        """Process images from the left camera."""
        self._positions[CameraPosition.LEFT], self._line_detected[CameraPosition.LEFT] = \
            self._process_image(msg, CameraPosition.LEFT)
        
    def _center_camera_callback(self, msg):
        """Process images from the center camera."""
        self._positions[CameraPosition.CENTER], self._line_detected[CameraPosition.CENTER] = \
            self._process_image(msg, CameraPosition.CENTER)
        
    def _right_camera_callback(self, msg):
        """Process images from the right camera."""
        self._positions[CameraPosition.RIGHT], self._line_detected[CameraPosition.RIGHT] = \
            self._process_image(msg, CameraPosition.RIGHT)
    
    def _process_image(self, img_msg, camera_position):
        """
        Process image to find the line position and detect if a line is present.
        
        Args:
            img_msg: ROS Image message
            camera_position: Which camera the image is from (left, center, right)
            
        Returns:
            tuple: (line_position, line_detected)
                line_position: normalized position (0.0 - 1.0)
                line_detected: boolean indicating if a line was found
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
            
            # Convert RGB to BGR
            if img_msg.encoding == 'rgb8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # Resize
            if cv_image.shape[0] != 16 or cv_image.shape[1] != 16:
                cv_image = cv2.resize(cv_image, (16, 16), interpolation=cv2.INTER_AREA)
            
            # Rotate
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            # Grayscale & blur
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (3, 3), 0)
            
            # Threshold
            threshold_value = self.get_parameter('threshold_value').value
            _, binary = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY_INV)
            
            # Contours
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Default values
            line_position = 0.5  # Default to center
            line_detected = False
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                min_area = self.get_parameter('min_contour_area').value
                
                if cv2.contourArea(largest_contour) > min_area:
                    M = cv2.moments(largest_contour)
                    
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        line_position = cx / cv_image.shape[1]
                        line_detected = True
                        
                        if self._debug_images:
                            self._publish_debug_image(binary, largest_contour, (cx, cy), 
                                                     line_position, camera_position)
            elif self._debug_images:
                debug_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                
                if self._detect_stop_line():
                    debug_img[0:2, 0:2] = [0, 0, 255]
                
                debug_img_large = cv2.resize(debug_img, (160, 160), interpolation=cv2.INTER_NEAREST)
                debug_msg = self._bridge.cv2_to_imgmsg(debug_img_large, encoding='bgr8')
                self._debug_publishers[camera_position].publish(debug_msg)
            
            return line_position, line_detected
            
        except Exception as e:
            self.get_logger().error(f'Error processing {camera_position} camera image: {str(e)}')
            return 0.5, False
    
    def _publish_debug_image(self, binary_img, contour, center_point, line_position, camera_position):
        """
        Create and publish a debug image showing line detection.
        
        Args:
            binary_img: Thresholded binary image
            contour: Detected contour
            center_point: (x, y) coordinates of contour center
            line_position: Normalized line position (0.0-1.0)
            camera_position: Which camera the image is from
        """
        debug_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(debug_img, [contour], 0, (0, 255, 0), 1)
        cv2.circle(debug_img, center_point, 1, (0, 0, 255), 1)
        
        pos_x = min(int(line_position * debug_img.shape[1]), debug_img.shape[1] - 1)
        cv2.line(debug_img, (pos_x, 0), (pos_x, 2), (0, 255, 255), 1)
        
        debug_img_large = cv2.resize(debug_img, (160, 160), interpolation=cv2.INTER_NEAREST)
        debug_msg = self._bridge.cv2_to_imgmsg(debug_img_large, encoding='bgr8')
        self._debug_publishers[camera_position].publish(debug_msg)
    
    def _detect_stop_line(self):
        """
        Detect if current line measurements indicate a stop line.
        
        Stop line is detected when all three cameras see a line at approximately
        the same position and that position is near the center.
        
        Returns:
            bool: True if a stop line is detected, False otherwise
        """
        # Check that all cameras detect a line
        if not all(self._line_detected.values()):
            return False
            
        max_deviation = self.get_parameter('stop_line_max_deviation').value
        center_min = self.get_parameter('stop_line_center_min').value
        center_max = self.get_parameter('stop_line_center_max').value
        
        # Calculate average position and deviations
        positions = list(self._positions.values())
        avg_pos = sum(positions) / len(positions)
        deviations = [abs(pos - avg_pos) for pos in positions]
        max_dev = max(deviations)
        
        if max_dev < max_deviation and center_min < avg_pos < center_max:
            return True
            
        return False

    def _detect_lost_line(self):
        """
        Detect if no lines are detected for at least 1 second.
        
        Returns:
            bool: True if no lines are detected for a sustained period, False otherwise
        """
        no_lines = not any(self._line_detected.values())
        current_time = self.get_clock().now()

        if no_lines:
            if self._no_line_start_time is None:
                self._no_line_start_time = current_time
            elif (current_time.nanoseconds - self._no_line_start_time.nanoseconds) > 1e9:
                return True
        else:
            self._no_line_start_time = None
            
        return False
    
    def _update_pid_terms(self, camera_position, error, dt_seconds):
        """
        Calculate PID terms for a camera.
        
        Args:
            camera_position: Which camera to calculate for
            error: Current error value
            dt_seconds: Time delta since last update
            
        Returns:
            tuple: (output, p_term, i_term, d_term)
        """
        # Get PID coefficients
        kp = self.get_parameter(f'{camera_position}_kp').value
        ki = self.get_parameter(f'{camera_position}_ki').value
        kd = self.get_parameter(f'{camera_position}_kd').value
        
        previous_error = self._previous_error[camera_position]
        integral = self._integral[camera_position]
        
        integral_windup_limit = self.get_parameter('integral_windup_limit').value
        integral += error * dt_seconds
        integral = max(min(integral, integral_windup_limit), -integral_windup_limit)
        
        derivative = (error - previous_error) / dt_seconds if dt_seconds > 0 else 0.0
        derivative_history = self._derivative_history[camera_position]
        derivative_history.append(derivative)
        filtered_derivative = sum(derivative_history) / len(derivative_history)
        
        self._previous_error[camera_position] = error
        self._integral[camera_position] = integral
        
        p_term = kp * error
        i_term = ki * integral
        d_term = kd * filtered_derivative
        
        output = p_term + i_term + d_term
        
        return output, p_term, i_term, d_term
    
    def _calculate_normalized_steering(self):
        """
        Calculate normalized steering value between -1.0 (full left) and +1.0 (full right).
        
        Returns:
            float: Normalized steering value
        """
        current_time = self.get_clock().now()
        dt = current_time - self._previous_time
        dt_seconds = dt.nanoseconds / 1e9
        self._previous_time = current_time
        
        dt_seconds = min(dt_seconds, 0.1)
        
        # Calculate error for each camera
        # For left camera: line should be at 0.0 (left edge)
        left_error = self._positions[CameraPosition.LEFT] - 0.0 if self._line_detected[CameraPosition.LEFT] else 0.0
        
        # For center camera: line should be at 0.5 (center)
        center_error = self._positions[CameraPosition.CENTER] - 0.5 if self._line_detected[CameraPosition.CENTER] else 0.0
        
        # For right camera: line should be at 1.0 (right edge)
        right_error = self._positions[CameraPosition.RIGHT] - 1.0 if self._line_detected[CameraPosition.RIGHT] else 0.0
        
        # Detect sharp turns
        corner_threshold = self.get_parameter('corner_detection_threshold').value
        sharp_turn_detected = self._is_sharp_turn(left_error, right_error, corner_threshold)
        
        # Calculate PID outputs
        outputs = {cam: 0.0 for cam in self._positions.keys()}
        
        if self._line_detected[CameraPosition.LEFT]:
            outputs[CameraPosition.LEFT], left_p, left_i, left_d = self._update_pid_terms(
                CameraPosition.LEFT, left_error, dt_seconds)
            self.get_logger().debug(f'Left PID: P={left_p:.2f}, I={left_i:.2f}, D={left_d:.2f}')
        
        if self._line_detected[CameraPosition.CENTER]:
            outputs[CameraPosition.CENTER], center_p, center_i, center_d = self._update_pid_terms(
                CameraPosition.CENTER, center_error, dt_seconds)
            self.get_logger().debug(f'Center PID: P={center_p:.2f}, I={center_i:.2f}, D={center_d:.2f}')
        
        if self._line_detected[CameraPosition.RIGHT]:
            outputs[CameraPosition.RIGHT], right_p, right_i, right_d = self._update_pid_terms(
                CameraPosition.RIGHT, right_error, dt_seconds)
            self.get_logger().debug(f'Right PID: P={right_p:.2f}, I={right_i:.2f}, D={right_d:.2f}')
        
        steering, total_weight = self._calculate_weighted_steering(outputs, sharp_turn_detected)
        
        # Normalize steering
        normalized_steering = np.tanh(steering)
        
        return normalized_steering
    
    def _is_sharp_turn(self, left_error, right_error, threshold):
        """
        Detect if the current readings indicate a sharp turn.
        
        Args:
            left_error: Error from left camera
            right_error: Error from right camera
            threshold: Threshold for corner detection
            
        Returns:
            bool: True if a sharp turn is detected
        """
        left_detected = self._line_detected[CameraPosition.LEFT]
        right_detected = self._line_detected[CameraPosition.RIGHT]
        
        if (left_detected and not right_detected and abs(left_error) > threshold) or \
           (right_detected and not left_detected and abs(right_error) > threshold) or \
           (left_detected and right_detected and abs(left_error - right_error) > 2 * threshold):
            return True
            
        return False
    
    def _calculate_weighted_steering(self, outputs, sharp_turn_detected):
        """
        Calculate weighted steering output based on detected lines.
        
        Args:
            outputs: Dict of PID outputs for each camera
            sharp_turn_detected: Whether a sharp turn was detected
            
        Returns:
            tuple: (steering, total_weight) - Weighted steering value and total weight
        """
        steering = 0.0
        total_weight = 0.0
        
        weights = {
            CameraPosition.LEFT: self.get_parameter(f'{CameraPosition.LEFT}_kp').value,
            CameraPosition.CENTER: self.get_parameter(f'{CameraPosition.CENTER}_kp').value,
            CameraPosition.RIGHT: self.get_parameter(f'{CameraPosition.RIGHT}_kp').value
        }
        
        if sharp_turn_detected:
            # Boost edge cameras' weights for sharp turns
            if self._line_detected[CameraPosition.LEFT] and not self._line_detected[CameraPosition.RIGHT]:
                weights[CameraPosition.LEFT] *= 1.5 
            elif self._line_detected[CameraPosition.RIGHT] and not self._line_detected[CameraPosition.LEFT]:
                weights[CameraPosition.RIGHT] *= 1.5
            
            # Reduce center camera influence during sharp turns
            weights[CameraPosition.CENTER] *= 0.7
        
        for camera, detected in self._line_detected.items():
            if detected:
                steering += outputs[camera] * weights[camera]
                total_weight += weights[camera]
        
        if total_weight > 0.0:
            steering /= total_weight
        
        # For sharp turns, apply boost to the steering command
        if sharp_turn_detected and abs(steering) > 0.1:
            steering *= 1.2
        
        return steering, total_weight
    
    def _publish_control(self):
        """Calculate and publish normalized steering value and detection results."""
        try:
            normalized_steering = self._calculate_normalized_steering()
            
            steering_msg = Float32()
            steering_msg.data = normalized_steering
            self._steering_pub.publish(steering_msg)
            
            stop_line_detected = self._detect_stop_line()
            stop_msg = Bool()
            stop_msg.data = stop_line_detected
            self._stop_line_pub.publish(stop_msg)

            lost_line_detected = self._detect_lost_line()
            lost_msg = Bool()
            lost_msg.data = lost_line_detected
            self._lost_line_pub.publish(lost_msg)
            
            if stop_line_detected:
                self.get_logger().debug('Stop line detected')
            if lost_line_detected:
                self.get_logger().info('Lost line detected')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing control: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = LinePIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()