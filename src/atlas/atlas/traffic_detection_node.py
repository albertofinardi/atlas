"""
ROS2 Traffic Sign Detector Node.

This node processes camera images to detect ArUco markers representing
traffic signs and lights using computer vision techniques.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import traceback
from enum import Enum


class TrafficSign(Enum):
    """Enumeration of traffic signs and lights."""
    LIGHT_RED = 0
    LIGHT_YELLOW = 1
    LIGHT_GREEN = 2
    SPEED_30 = 3
    SPEED_50 = 4


class TrafficDetectorNode(Node):
    """
    ROS2 node that detects traffic markers using ArUco markers.
    
    This node:
    - Subscribes to camera images
    - Processes images to detect ArUco markers
    - Identifies traffic signs based on marker IDs
    - Publishes detected traffic sign information
    - Provides debug visualization if enabled
    """
    
    def __init__(self):
        """Initialize the traffic detector node."""
        super().__init__('traffic_detection_node')
        
        # Initialize CV bridge
        self._bridge = CvBridge()
        
        # Declare parameters
        self._declare_parameters()
        
        # Set up publisher and subscriber
        self._setup_communication()
        
        # Configure ArUco detector
        self._setup_aruco_detector()
        
        # Initialize detection state
        self._detection_state = {
            'current_sign': None,
            'confidence': 0,
            'last_id': None
        }
        
        # Initialize debug publishers if debug mode is enabled
        self._setup_debug_mode()
        
        self.get_logger().info('Traffic Marker Detector Node initialized')

    def _declare_parameters(self):
        """Declare all node parameters."""
        # General parameters
        self.declare_parameter('debug', False)
        
        # Detection parameters
        self.declare_parameter('min_threshold', 40)
        self.declare_parameter('max_threshold', 150)
        self.declare_parameter('threshold_step', 20)
        self.declare_parameter('min_detection_confidence', 10)
        self.declare_parameter('marker_stability_threshold', 5)

    def _setup_communication(self):
        """Set up ROS publishers and subscribers."""
        # Subscribe to camera image
        self.create_subscription(
            Image,
            'camera/image_color',
            self._camera_callback,
            10
        )
        
        # Publisher for detected traffic signs
        self._marker_publisher = self.create_publisher(
            String,
            'traffic/id',
            10
        )
    
    def _setup_aruco_detector(self):
        """Set up ArUco marker detector."""
        # Initialize ArUco dictionary and parameters
        self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self._aruco_params = cv2.aruco.DetectorParameters_create()
        self._configure_aruco_parameters()
        
        # Define mapping between marker IDs and traffic signs
        self._sign_mapping = {sign.value: sign.name for sign in TrafficSign}
    
    def _configure_aruco_parameters(self):
        """Configure ArUco detector parameters for optimal marker detection."""
        params = self._aruco_params
        params.adaptiveThreshConstant = 7
        params.minMarkerPerimeterRate = 0.03
        params.maxMarkerPerimeterRate = 1.0
        params.polygonalApproxAccuracyRate = 0.05
        params.minCornerDistanceRate = 0.05
        params.minDistanceToBorder = 3
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.errorCorrectionRate = 1.0

    def _setup_debug_mode(self):
        """Set up debug visualization if enabled."""
        self._debug_enabled = self.get_parameter('debug').value
        
        if self._debug_enabled:
            # Publishers for debug images
            self._debug_image_publisher = self.create_publisher(
                Image,
                'traffic/debug/overlay',
                10
            )
            self._processed_image_publisher = self.create_publisher(
                Image,
                'traffic/debug/processed',
                10
            )
            self.get_logger().info('Debug visualization enabled')

    def _camera_callback(self, msg):
        """
        Process incoming camera images and detect traffic markers.
        
        Args:
            msg: ROS Image message from camera
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Convert RGB to BGR if needed
            if msg.encoding == 'rgb8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # Get parameters for marker detection
            min_threshold = self.get_parameter('min_threshold').value
            max_threshold = self.get_parameter('max_threshold').value
            threshold_step = self.get_parameter('threshold_step').value
            min_confidence = self.get_parameter('min_detection_confidence').value
            
            # Detect markers using multiple thresholds
            detected_markers = self._detect_markers_with_multi_threshold(
                cv_image, min_threshold, max_threshold, threshold_step)
            
            # Process results with or without debug visualization
            if self._debug_enabled:
                debug_image = cv_image.copy()
                processed_vis = self._create_processed_visualization(cv_image)
                
                self._process_detection_results(
                    detected_markers, debug_image, processed_vis, min_confidence, True)
                
                self._publish_debug_images(debug_image, processed_vis)
            else:
                self._process_detection_results(
                    detected_markers, None, None, min_confidence, False)
                
        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {str(e)}')
            self.get_logger().error(traceback.format_exc())

    def _process_detection_results(self, detected_markers, debug_image, processed_vis, 
                                   min_confidence, create_visuals):
        """
        Process marker detection results and update tracking.
        
        Args:
            detected_markers: List of detected marker information
            debug_image: Original image with debug overlay (or None)
            processed_vis: Processed binary image visualization (or None)
            min_confidence: Minimum confidence required for detection
            create_visuals: Whether to create visual output
        """
        if detected_markers:
            # Sort markers by size (largest first)
            sorted_markers = sorted(detected_markers, key=lambda x: x['size'], reverse=True)
            largest_marker = sorted_markers[0]
            
            # Create visual output if requested
            if create_visuals:
                self._highlight_all_markers(debug_image, processed_vis, sorted_markers)
            
            # Update detection confidence
            if self._detection_state['last_id'] == largest_marker['id']:
                self._detection_state['confidence'] += 1
            else:
                self._detection_state['confidence'] = 1
                self._detection_state['last_id'] = largest_marker['id']
            
            # Publish when confidence threshold is reached
            if self._detection_state['confidence'] >= min_confidence:
                if largest_marker['type'] != self._detection_state['current_sign']:
                    self._detection_state['current_sign'] = largest_marker['type']
                    
                    # Publish the detected sign
                    sign_msg = String()
                    sign_msg.data = self._detection_state['current_sign']
                    self._marker_publisher.publish(sign_msg)
                    
                    self.get_logger().info(
                        f"Detected marker ID {largest_marker['id']} "
                        f"({self._detection_state['current_sign']})")
        else:
            # Decrease confidence when no markers are detected
            self._detection_state['confidence'] = max(0, self._detection_state['confidence'] - 1)
            
            # Add indicators for no detection in debug mode
            if create_visuals:
                self._add_no_detection_indicators(debug_image, processed_vis)
        
        # Add detection status information in debug mode
        if create_visuals:
            self._add_detection_status(debug_image, processed_vis)

    def _detect_markers_with_multi_threshold(self, image, min_threshold, max_threshold, step):
        """
        Try multiple thresholds to find the best one for ArUco detection.
        
        Args:
            image: Input color image
            min_threshold: Minimum threshold value to try
            max_threshold: Maximum threshold value to try
            step: Step size between thresholds
            
        Returns:
            list: Filtered list of unique detected markers
        """
        # Convert to grayscale and apply blur for better detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        all_detected_markers = []
        best_markers = []
        best_threshold = None
        
        # Try different threshold values
        for threshold in range(min_threshold, max_threshold + 1, step):
            # Apply binary threshold
            _, binary = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY)
            
            # Apply morphological opening to remove noise
            kernel = np.ones((3, 3), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
            
            # Detect ArUco markers
            corners, ids, _ = cv2.aruco.detectMarkers(
                binary, self._aruco_dict, parameters=self._aruco_params)
            
            # Process detection results if markers were found
            if ids is not None and len(ids) > 0:
                markers_info = self._process_aruco_detection(corners, ids, threshold)
                
                # Keep track of best threshold (most markers)
                if len(markers_info) > len(best_markers):
                    best_markers = markers_info
                    best_threshold = threshold
                
                all_detected_markers.extend(markers_info)
        
        # Log best threshold if any markers were detected
        if best_threshold is not None:
            self.get_logger().debug(
                f"Best threshold: {best_threshold}, detected {len(best_markers)} markers")
        
        # Filter out duplicate markers
        return self._filter_unique_markers(all_detected_markers)

    def _filter_unique_markers(self, markers):
        """
        Remove duplicate markers, keeping the largest instance of each ID.
        
        Args:
            markers: List of marker information dictionaries
            
        Returns:
            list: Filtered list with one instance per marker ID
        """
        unique_markers = {}
        
        for marker in markers:
            marker_id = marker['id']
            if marker_id not in unique_markers or marker['size'] > unique_markers[marker_id]['size']:
                unique_markers[marker_id] = marker
        
        return list(unique_markers.values())

    def _process_aruco_detection(self, corners, ids, threshold):
        """
        Process ArUco detection results into a unified format.
        
        Args:
            corners: List of detected marker corners
            ids: List of detected marker IDs
            threshold: Threshold value used for detection
            
        Returns:
            list: List of marker information dictionaries
        """
        marker_info = []
        
        for i in range(len(ids)):
            marker_id = ids[i][0]
            marker_corners = corners[i][0]
            
            # Calculate perimeter as a measure of marker size
            perimeter = cv2.arcLength(marker_corners, True)
            
            # Calculate center point
            center_x = int(np.mean([p[0] for p in marker_corners]))
            center_y = int(np.mean([p[1] for p in marker_corners]))
            
            # Look up sign type from mapping or use default
            sign_type = self._sign_mapping.get(marker_id, f"ID:{marker_id}")
            
            # Create marker information dictionary
            marker_info.append({
                'id': marker_id,
                'type': sign_type,
                'size': perimeter,
                'center_x': center_x,
                'center_y': center_y,
                'corners': marker_corners,
                'threshold': threshold
            })
        
        return marker_info

    def _create_processed_visualization(self, image):
        """
        Create a visualization of the processed binary image.
        
        Args:
            image: Original input image
            
        Returns:
            numpy.ndarray: Visualization of processed binary image
        """
        # Convert to grayscale and apply blur
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Use middle threshold value for visualization
        threshold_value = (self.get_parameter('min_threshold').value + 
                          self.get_parameter('max_threshold').value) // 2
        
        # Apply threshold
        _, binary = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY)
        
        # Apply morphological opening
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # Create RGB visualization from binary image
        processed_vis = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
        processed_vis[:, :, 0] = binary
        processed_vis[:, :, 1] = binary
        processed_vis[:, :, 2] = binary
        
        return processed_vis

    def _highlight_all_markers(self, debug_image, processed_vis, sorted_markers):
        """
        Highlight all detected markers in debug images.
        
        Args:
            debug_image: Original image with overlay
            processed_vis: Processed binary image visualization
            sorted_markers: List of detected markers sorted by size
        """
        if not sorted_markers:
            return
            
        largest_marker = sorted_markers[0]
        
        # Highlight secondary markers in blue
        for marker in sorted_markers[1:]:
            self._highlight_marker(debug_image, marker, (255, 0, 0), 1)
            self._highlight_marker(processed_vis, marker, (255, 0, 0), 1)
        
        # Highlight largest (primary) marker in green
        self._highlight_marker(debug_image, largest_marker, (0, 255, 0), 2)
        self._highlight_marker(processed_vis, largest_marker, (0, 255, 0), 2)

    def _highlight_marker(self, image, marker, color, thickness):
        """
        Highlight a marker in the image with polygon, center, and label.
        
        Args:
            image: Image to draw on
            marker: Marker information dictionary
            color: BGR color tuple
            thickness: Line thickness
        """
        # Draw polygon around marker
        corners = np.array([marker['corners']], dtype=np.float32)
        cv2.polylines(image, [corners.astype(np.int32)], True, color, thickness)
        
        # Draw center point
        cv2.circle(image, (marker['center_x'], marker['center_y']), 3, color, -1)
        
        # Add label for marker type
        cv2.putText(image, marker['type'], 
                   (marker['center_x'] - 20, marker['center_y'] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

    def _add_no_detection_indicators(self, debug_image, processed_vis):
        """
        Add indicators when no markers are detected.
        
        Args:
            debug_image: Original image with overlay
            processed_vis: Processed binary image visualization
        """
        message = "NO MARKERS DETECTED"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_color = (0, 0, 255)
        font_thickness = 1
        
        # Center text horizontally
        text_size = cv2.getTextSize(message, font, font_scale, font_thickness)[0]
        x_pos_debug = int(debug_image.shape[1]/2 - text_size[0]/2)
        x_pos_processed = int(processed_vis.shape[1]/2 - text_size[0]/2)
        
        # Add text to both images
        cv2.putText(debug_image, message, (x_pos_debug, 30), font, 
                    font_scale, font_color, font_thickness)
        cv2.putText(processed_vis, message, (x_pos_processed, 60), font, 
                    font_scale, font_color, font_thickness)

    def _add_detection_status(self, debug_image, processed_vis):
        """
        Add detection status information to debug images.
        
        Args:
            debug_image: Original image with overlay
            processed_vis: Processed binary image visualization
        """
        current_sign = self._detection_state['current_sign']
        status_text = f"Current: {current_sign if current_sign else 'None'}"
        confidence_text = f"Confidence: {self._detection_state['confidence']}"
        
        # Add status text to both images
        cv2.putText(debug_image, status_text, (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(processed_vis, status_text, (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        # Add confidence text
        cv2.putText(debug_image, confidence_text, (10, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(processed_vis, confidence_text, (10, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    def _publish_debug_images(self, debug_image, processed_vis):
        """
        Publish debug visualization images.
        
        Args:
            debug_image: Original image with overlay
            processed_vis: Processed binary image visualization
        """
        # Convert OpenCV images to ROS messages and publish
        debug_msg = self._bridge.cv2_to_imgmsg(debug_image, "bgr8")
        self._debug_image_publisher.publish(debug_msg)
        
        processed_msg = self._bridge.cv2_to_imgmsg(processed_vis, "bgr8")
        self._processed_image_publisher.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()