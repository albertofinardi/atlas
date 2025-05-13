import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque


class LinePIDNode(Node):
    def __init__(self):
        super().__init__('line_detection_node')
        
        # Create publisher for normalized steering (-1.0 to +1.0)
        self.steering_pub = self.create_publisher(
            Float32, 'line_pid/steering', 10)
            
        # Create publisher for stop line detection
        self.stop_line_pub = self.create_publisher(
            Bool, 'line_pid/stop_line_detected', 10)
        
        self.left_cam_sub = self.create_subscription(
            Image, '/vision/left', self.left_camera_callback, 10)
        self.center_cam_sub = self.create_subscription(
            Image, '/vision/center', self.center_camera_callback, 10)
        self.right_cam_sub = self.create_subscription(
            Image, '/vision/right', self.right_camera_callback, 10)
        
        self.bridge = CvBridge()
        
        # Store the latest processed positions (0.0 = left edge, 0.5 = center, 1.0 = right edge)
        self.left_position = 0.5
        self.center_position = 0.5
        self.right_position = 0.5
        
        self.left_line_detected = False
        self.center_line_detected = False
        self.right_line_detected = False
        
        self.publish_timer = self.create_timer(1/30, self.publish_control)
        
        self.declare_parameter('threshold_value', 128)
        self.declare_parameter('min_contour_area', 3)
        self.declare_parameter('debug_images', True)
        
        self.declare_parameter('stop_line_max_deviation', 0.15)
        self.declare_parameter('stop_line_center_min', 0.4)
        self.declare_parameter('stop_line_center_max', 0.6)
        
        self.declare_parameter('left_kp', 0.8)
        self.declare_parameter('left_ki', 0.02)
        self.declare_parameter('left_kd', 0.35)
        
        self.declare_parameter('center_kp', 1.0)
        self.declare_parameter('center_ki', 0.02)
        self.declare_parameter('center_kd', 0.4)
        
        self.declare_parameter('right_kp', 0.8)
        self.declare_parameter('right_ki', 0.02)
        self.declare_parameter('right_kd', 0.35)
        
        self.declare_parameter('integral_windup_limit', 0.2)
        self.declare_parameter('corner_detection_threshold', 0.25)
        
        self.left_integral = 0.0
        self.center_integral = 0.0
        self.right_integral = 0.0
        
        self.left_previous_error = 0.0
        self.center_previous_error = 0.0
        self.right_previous_error = 0.0
        
        self.previous_time = self.get_clock().now()
        
        self.derivative_filter_size = 3
        self.left_derivative_history = deque([0.0] * self.derivative_filter_size, maxlen=self.derivative_filter_size)
        self.center_derivative_history = deque([0.0] * self.derivative_filter_size, maxlen=self.derivative_filter_size)
        self.right_derivative_history = deque([0.0] * self.derivative_filter_size, maxlen=self.derivative_filter_size)
        
        # For debugging: publish processed images
        self.debug_images = self.get_parameter('debug_images').value
        if self.debug_images:
            self.left_debug_pub = self.create_publisher(
                Image, 'line_pid/debug/left_cam', 10)
            self.center_debug_pub = self.create_publisher(
                Image, 'line_pid/debug/center_cam', 10)
            self.right_debug_pub = self.create_publisher(
                Image, 'line_pid/debug/right_cam', 10)
        
        self.get_logger().info('Line Detection Node initialized with normalized steering output (-1.0 to +1.0)')

    def left_camera_callback(self, msg):
        self.left_position, self.left_line_detected = self.process_image(msg, 'left')
        
    def center_camera_callback(self, msg):
        self.center_position, self.center_line_detected = self.process_image(msg, 'center')
        
    def right_camera_callback(self, msg):
        self.right_position, self.right_line_detected = self.process_image(msg, 'right')
    
    def process_image(self, img_msg, camera_name):
        """Process image to find the line position and detect if a line is present."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
            if img_msg.encoding == 'rgb8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            if cv_image.shape[0] != 16 or cv_image.shape[1] != 16:
                cv_image = cv2.resize(cv_image, (16, 16), interpolation=cv2.INTER_AREA)
            
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            blurred = cv2.GaussianBlur(gray, (3, 3), 0)
            
            threshold_value = self.get_parameter('threshold_value').value
            _, binary = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY_INV)
            
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            line_position = 0.5 # Default to center
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
                        
                        if self.debug_images:
                            debug_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                            cv2.drawContours(debug_img, [largest_contour], 0, (0, 255, 0), 1)
                            cv2.circle(debug_img, (cx, cy), 1, (0, 0, 255), 1)
                            
                            pos_x = min(int(line_position * debug_img.shape[1]), debug_img.shape[1] - 1)
                            cv2.line(debug_img, (pos_x, 0), (pos_x, 2), (0, 255, 255), 1)
                            
                            debug_img_large = cv2.resize(debug_img, (160, 160), interpolation=cv2.INTER_NEAREST)
                            
                            debug_msg = self.bridge.cv2_to_imgmsg(debug_img_large, encoding='bgr8')
                            
                            if camera_name == 'left':
                                self.left_debug_pub.publish(debug_msg)
                            elif camera_name == 'center':
                                self.center_debug_pub.publish(debug_msg)
                            elif camera_name == 'right':
                                self.right_debug_pub.publish(debug_msg)
            
            elif self.debug_images:
                debug_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                
                if self.detect_stop_line():
                    debug_img[0:2, 0:2] = [0, 0, 255]
                
                debug_img_large = cv2.resize(debug_img, (160, 160), interpolation=cv2.INTER_NEAREST)
                
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img_large, encoding='bgr8')
                
                if camera_name == 'left':
                    self.left_debug_pub.publish(debug_msg)
                elif camera_name == 'center':
                    self.center_debug_pub.publish(debug_msg)
                elif camera_name == 'right':
                    self.right_debug_pub.publish(debug_msg)
            
            return line_position, line_detected
            
        except Exception as e:
            self.get_logger().error(f'Error processing {camera_name} camera image: {str(e)}')
            return 0.5, False
    
    def detect_stop_line(self):
        """Detect if current line measurements indicate a stop line."""
        if not (self.left_line_detected and self.center_line_detected and self.right_line_detected):
            return False
            
        max_deviation = self.get_parameter('stop_line_max_deviation').value
        center_min = self.get_parameter('stop_line_center_min').value
        center_max = self.get_parameter('stop_line_center_max').value
        
        avg_pos = (self.left_position + self.center_position + self.right_position) / 3.0
        
        deviations = [
            abs(self.left_position - avg_pos),
            abs(self.center_position - avg_pos),
            abs(self.right_position - avg_pos)
        ]
        max_dev = max(deviations)
        
        if max_dev < max_deviation and center_min < avg_pos < center_max:
            return True
            
        return False
    
    def update_pid_terms(self, camera, error, dt_seconds):
        """Calculate PID terms for a camera"""
        kp = self.get_parameter(f'{camera}_kp').value
        ki = self.get_parameter(f'{camera}_ki').value
        kd = self.get_parameter(f'{camera}_kd').value
        
        previous_error = getattr(self, f'{camera}_previous_error')
        integral = getattr(self, f'{camera}_integral')
        
        integral_windup_limit = self.get_parameter('integral_windup_limit').value
        integral += error * dt_seconds

        if integral > integral_windup_limit:
            integral = integral_windup_limit
        elif integral < -integral_windup_limit:
            integral = -integral_windup_limit
        
        derivative = (error - previous_error) / dt_seconds if dt_seconds > 0 else 0.0

        derivative_history = getattr(self, f'{camera}_derivative_history')
        derivative_history.append(derivative)
        filtered_derivative = sum(derivative_history) / len(derivative_history)
        
        setattr(self, f'{camera}_previous_error', error)
        setattr(self, f'{camera}_integral', integral)
        
        p_term = kp * error
        i_term = ki * integral
        d_term = kd * filtered_derivative
        
        output = p_term + i_term + d_term
        
        return output, p_term, i_term, d_term
    
    def calculate_normalized_steering(self):
        """Calculate normalized steering value between -1.0 (full left) and +1.0 (full right)"""
        current_time = self.get_clock().now()
        dt = current_time - self.previous_time
        dt_seconds = dt.nanoseconds / 1e9
        self.previous_time = current_time
        
        if dt_seconds > 0.1:
            dt_seconds = 0.1
        
        left_error = self.left_position - 0.0 if self.left_line_detected else 0.0
        
        # For center camera: line should be at 0.5 (center)
        center_error = self.center_position - 0.5 if self.center_line_detected else 0.0
        
        # For right camera: line should be at 1.0 (right edge)
        right_error = self.right_position - 1.0 if self.right_line_detected else 0.0
        
        sharp_turn_detected = False
        corner_threshold = self.get_parameter('corner_detection_threshold').value
        
        if (self.left_line_detected and not self.right_line_detected and abs(left_error) > corner_threshold) or \
           (self.right_line_detected and not self.left_line_detected and abs(right_error) > corner_threshold) or \
           (self.left_line_detected and self.right_line_detected and abs(left_error - right_error) > 2*corner_threshold):
            sharp_turn_detected = True
        
        left_output = 0.0
        center_output = 0.0
        right_output = 0.0
        
        if self.left_line_detected:
            left_output, left_p, left_i, left_d = self.update_pid_terms('left', left_error, dt_seconds)
            self.get_logger().debug(f'Left PID: P={left_p:.2f}, I={left_i:.2f}, D={left_d:.2f}')
        
        if self.center_line_detected:
            center_output, center_p, center_i, center_d = self.update_pid_terms('center', center_error, dt_seconds)
            self.get_logger().debug(f'Center PID: P={center_p:.2f}, I={center_i:.2f}, D={center_d:.2f}')
        
        if self.right_line_detected:
            right_output, right_p, right_i, right_d = self.update_pid_terms('right', right_error, dt_seconds)
            self.get_logger().debug(f'Right PID: P={right_p:.2f}, I={right_i:.2f}, D={right_d:.2f}')
        
        steering = 0.0
        total_weight = 0.0
        
        left_weight = self.get_parameter('left_kp').value
        center_weight = self.get_parameter('center_kp').value
        right_weight = self.get_parameter('right_kp').value
        
        if sharp_turn_detected:
            # Boost edge cameras' weights for sharp turns
            if self.left_line_detected and not self.right_line_detected:
                left_weight *= 1.5 
            elif self.right_line_detected and not self.left_line_detected:
                right_weight *= 1.5
            
            # Reduce center camera influence during sharp turns
            center_weight *= 0.7
        
        if self.left_line_detected:
            steering += left_output * left_weight
            total_weight += left_weight
        
        if self.center_line_detected:
            steering += center_output * center_weight
            total_weight += center_weight
        
        if self.right_line_detected:
            steering += right_output * right_weight
            total_weight += right_weight
        
        # Normalize by total weight if any line was detected
        if total_weight > 0.0:
            steering /= total_weight
        
        # For sharp turns, apply boost to the steering command
        if sharp_turn_detected and abs(steering) > 0.1:
            steering *= 1.2
        
        # Normalize steering to range [-1.0, 1.0]
        normalized_steering = np.tanh(steering)
        
        return normalized_steering
    
    def publish_control(self):
        """Calculate and publish normalized steering value and stop line detection."""
        try:
            normalized_steering = self.calculate_normalized_steering()
            
            steering_msg = Float32()
            steering_msg.data = normalized_steering
            self.steering_pub.publish(steering_msg)
            
            stop_line_detected = self.detect_stop_line()
            
            stop_msg = Bool()
            stop_msg.data = stop_line_detected
            self.stop_line_pub.publish(stop_msg)
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