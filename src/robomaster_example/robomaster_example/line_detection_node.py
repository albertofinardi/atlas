#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineDetectionNode(Node):
    def __init__(self):
        super().__init__('line_detection_node')
        
        # Create publisher for line detection percentages (position values)
        self.percentages_pub = self.create_publisher(
            Float32MultiArray, 'line_detection/positions', 10)
            
        # Create publisher for stop line detection
        self.stop_line_pub = self.create_publisher(
            Bool, 'line_detection/stop_line_detected', 10)
        
        # Create subscribers for the three camera images
        self.left_cam_sub = self.create_subscription(
            Image, '/vision/left', self.left_camera_callback, 10)
        self.center_cam_sub = self.create_subscription(
            Image, '/vision/center', self.center_camera_callback, 10)
        self.right_cam_sub = self.create_subscription(
            Image, '/vision/right', self.right_camera_callback, 10)
        
        # Bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        
        # Store the latest processed positions (0.0 = left edge, 0.5 = center, 1.0 = right edge)
        self.left_position = 0.5
        self.center_position = 0.5
        self.right_position = 0.5
        
        # Store if a line is detected in each camera
        self.left_line_detected = False
        self.center_line_detected = False
        self.right_line_detected = False
        
        # Create timer for publishing combined data (30Hz)
        self.publish_timer = self.create_timer(1/30, self.publish_positions)
        
        # Declare parameters for line detection
        self.declare_parameter('threshold_value', 128)
        self.declare_parameter('min_contour_area', 100)  # Minimum size of line to detect
        self.declare_parameter('debug_images', True)
        
        # Parameters for stop line detection
        self.declare_parameter('stop_line_max_deviation', 0.15)
        self.declare_parameter('stop_line_center_min', 0.4)
        self.declare_parameter('stop_line_center_max', 0.6)
        
        # Parameter for camera orientation
        
        # For debugging: optionally publish processed images
        self.debug_images = self.get_parameter('debug_images').value
        if self.debug_images:
            self.left_debug_pub = self.create_publisher(
                Image, 'line_detection/debug/left', 10)
            self.center_debug_pub = self.create_publisher(
                Image, 'line_detection/debug/center', 10)
            self.right_debug_pub = self.create_publisher(
                Image, 'line_detection/debug/right', 10)
        
        self.get_logger().info('Line Detection Node initialized')

    def left_camera_callback(self, msg):
        self.left_position, self.left_line_detected = self.process_image(msg, 'left')
        
    def center_camera_callback(self, msg):
        self.center_position, self.center_line_detected = self.process_image(msg, 'center')
        
    def right_camera_callback(self, msg):
        self.right_position, self.right_line_detected = self.process_image(msg, 'right')
    
    def process_image(self, img_msg, camera_name):
        """Process image to find the line position and detect if a line is present."""
        try:
            # Convert ROS Image message to OpenCV image
            # Note: The camera publishes in 'rgb8' format but OpenCV uses BGR
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
            # Convert RGB to BGR if needed
            if img_msg.encoding == 'rgb8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # Check if we need to rotate the image:
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Apply thresholding to isolate the black line
            threshold_value = self.get_parameter('threshold_value').value
            _, binary = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY_INV)
            
            # Find contours in the binary image
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Initialize values
            line_position = 2.0  # Default to center
            line_detected = False
            
            # Find the largest contour, which should be the line
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                min_area = self.get_parameter('min_contour_area').value
                
                if cv2.contourArea(largest_contour) > min_area:
                    # Calculate the centroid of the line
                    M = cv2.moments(largest_contour)
                    
                    if M["m00"] > 0:
                        # Calculate centroid coordinates
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Determine line position based on camera orientation
                        line_position = cx / cv_image.shape[1]
                        
                        line_detected = True
                        
                        # Draw contour and centroid for debugging
                        if self.debug_images:
                            debug_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                            cv2.drawContours(debug_img, [largest_contour], 0, (0, 255, 0), 2)
                            cv2.circle(debug_img, (cx, int(M["m01"] / M["m00"])), 7, (0, 0, 255), -1)
                            
                            # Add text with position value
                            text = f"Pos: {line_position:.2f}"
                            cv2.putText(debug_img, text, (20, 30), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                            
                            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                            
                            if camera_name == 'left':
                                self.left_debug_pub.publish(debug_msg)
                            elif camera_name == 'center':
                                self.center_debug_pub.publish(debug_msg)
                            elif camera_name == 'right':
                                self.right_debug_pub.publish(debug_msg)
            
            # If no line is detected and debug is enabled, publish the thresholded image
            elif self.debug_images:
                debug_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                text = "No line detected"
                cv2.putText(debug_img, text, (20, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # Check for stop line and add to debug image if it's being detected
                if self.detect_stop_line():
                    stop_text = "STOP LINE DETECTED"
                    cv2.putText(debug_img, stop_text, (20, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                
                if camera_name == 'left':
                    self.left_debug_pub.publish(debug_msg)
                elif camera_name == 'center':
                    self.center_debug_pub.publish(debug_msg)
                elif camera_name == 'right':
                    self.right_debug_pub.publish(debug_msg)
            
            return line_position, line_detected
            
        except Exception as e:
            self.get_logger().error(f'Error processing {camera_name} camera image: {str(e)}')
            return 0.5, False  # Default to center, no line detected on error
    
    def detect_stop_line(self):
        """Detect if current line measurements indicate a stop line."""
        # Check if all three cameras detect a line
        if not (self.left_line_detected and self.center_line_detected and self.right_line_detected):
            return False
            
        # Get parameters for stop line detection
        max_deviation = self.get_parameter('stop_line_max_deviation').value
        center_min = self.get_parameter('stop_line_center_min').value
        center_max = self.get_parameter('stop_line_center_max').value
        
        # Calculate the average position
        avg_pos = (self.left_position + self.center_position + self.right_position) / 3.0
        
        # Calculate maximum deviation from average
        deviations = [
            abs(self.left_position - avg_pos),
            abs(self.center_position - avg_pos),
            abs(self.right_position - avg_pos)
        ]
        max_dev = max(deviations)
        
        # Check if positions are similar (small deviation) and near center
        if max_dev < max_deviation and center_min < avg_pos < center_max:
            return True
            
        return False
    
    def publish_positions(self):
        """Publish line positions and detection status for all three cameras."""
        try:
            # Create and publish the combined message for line positions
            msg = Float32MultiArray()
            # Format: [left_pos, center_pos, right_pos, left_detected, center_detected, right_detected]
            # Using 1.0 for detected, 0.0 for not detected
            msg.data = [
                self.left_position, 
                self.center_position, 
                self.right_position,
                1.0 if self.left_line_detected else 0.0,
                1.0 if self.center_line_detected else 0.0,
                1.0 if self.right_line_detected else 0.0
            ]
            self.percentages_pub.publish(msg)
            
            # Detect if we have a stop line
            stop_line_detected = self.detect_stop_line()
            
            # Publish stop line detection
            stop_msg = Bool()
            stop_msg.data = stop_line_detected
            self.stop_line_pub.publish(stop_msg)
            
            # Log data occasionally for debugging (use debug level to avoid console spam)
            self.get_logger().debug(
                f'Line positions: L={self.left_position:.2f}({self.left_line_detected}), ' + 
                f'C={self.center_position:.2f}({self.center_line_detected}), ' + 
                f'R={self.right_position:.2f}({self.right_line_detected}), ' +
                f'Stop line: {stop_line_detected}'
            )
        except Exception as e:
            self.get_logger().error(f'Error publishing positions: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()