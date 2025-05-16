#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrafficDetectorNode(Node):
    def __init__(self):
        super().__init__('traffic_detection_node')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            'camera/image_color',
            self.camera_callback,
            10
        )
        
        self.marker_publisher = self.create_publisher(
            String,
            'traffic/id',
            10
        )
        
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.configure_aruco_parameters()
        
        # Define sign mapping
        self.sign_mapping = {
            0: "LIGHT_RED",
            1: "LIGHT_YELLOW",
            2: "LIGHT_GREEN",
            3: "SPEED_30",
            4: "SPEED_50"
        }
        
        self.current_detected_sign = None
        self.detection_confidence = 0
        self.last_detected_id = None
        
        self.declare_node_parameters()
        
        if self.get_parameter('enable_debug_visualization').value:
            self.debug_image_publisher = self.create_publisher(
                Image,
                'traffic/debug/overlay',
                10
            )
            self.processed_image_publisher = self.create_publisher(
                Image,
                'traffic/debug/processed',
                10
            )
            self.get_logger().info('Debug visualization enabled')
        
        self.get_logger().info('Traffic Marker Detector Node initialized')

    def declare_node_parameters(self):
        """Declare all node parameters"""
        self.declare_parameter('enable_debug_visualization', True)
        self.declare_parameter('min_threshold', 40)
        self.declare_parameter('max_threshold', 150)
        self.declare_parameter('threshold_step', 20)
        self.declare_parameter('min_detection_confidence', 3)
        self.declare_parameter('marker_stability_threshold', 5)

    def configure_aruco_parameters(self):
        """Configure ArUco detector parameters for white-on-black markers"""
        params = self.aruco_params
        params.adaptiveThreshConstant = 7
        params.minMarkerPerimeterRate = 0.03
        params.maxMarkerPerimeterRate = 1.0
        params.polygonalApproxAccuracyRate = 0.05
        params.minCornerDistanceRate = 0.05
        params.minDistanceToBorder = 3
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.errorCorrectionRate = 1.0

    def camera_callback(self, msg):
        """Process incoming camera images and detect traffic markers"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if msg.encoding == 'rgb8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            enable_debug = self.get_parameter('enable_debug_visualization').value
            
            min_threshold = self.get_parameter('min_threshold').value
            max_threshold = self.get_parameter('max_threshold').value
            threshold_step = self.get_parameter('threshold_step').value
            min_confidence = self.get_parameter('min_detection_confidence').value
            
            detected_markers = self.detect_markers_with_multi_threshold(
                cv_image, min_threshold, max_threshold, threshold_step)
            
            if enable_debug and hasattr(self, 'debug_image_publisher'):
                debug_image = cv_image.copy()
                
                processed_vis = self.create_processed_visualization(cv_image, detected_markers)
                
                self.process_detection_results(detected_markers, debug_image, processed_vis, min_confidence, True)
                
                self.publish_debug_images(debug_image, processed_vis)
            else:
                self.process_detection_results(detected_markers, None, None, min_confidence, False)
                
        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def process_detection_results(self, detected_markers, debug_image, processed_vis, min_confidence, create_visuals):
        """Process marker detection results and update tracking"""
        if detected_markers:
            sorted_markers = sorted(detected_markers, key=lambda x: x['size'], reverse=True)
            largest_marker = sorted_markers[0]
            
            if create_visuals:
                self.highlight_all_markers(debug_image, processed_vis, sorted_markers)
            
            if self.last_detected_id == largest_marker['id']:
                self.detection_confidence += 1
            else:
                self.detection_confidence = 1
                self.last_detected_id = largest_marker['id']
            
            if self.detection_confidence >= min_confidence:
                if largest_marker['type'] != self.current_detected_sign:
                    self.current_detected_sign = largest_marker['type']
                    
                    sign_msg = String()
                    sign_msg.data = self.current_detected_sign
                    self.marker_publisher.publish(sign_msg)
                    
                    self.get_logger().info(
                        f"Detected marker ID {largest_marker['id']} ({self.current_detected_sign})")
        else:
            self.detection_confidence = max(0, self.detection_confidence - 1)
            
            if create_visuals:
                self.add_no_detection_indicators(debug_image, processed_vis)
        
        if create_visuals:
            self.add_detection_status(debug_image, processed_vis)

    def highlight_all_markers(self, debug_image, processed_vis, sorted_markers):
        """Highlight all detected markers in debug images"""
        if not sorted_markers:
            return
            
        largest_marker = sorted_markers[0]
        
        for marker in sorted_markers[1:]:
            self.highlight_marker(debug_image, marker, (255, 0, 0), 1)
            self.highlight_marker(processed_vis, marker, (255, 0, 0), 1)
        
        self.highlight_marker(debug_image, largest_marker, (0, 255, 0), 2)
        self.highlight_marker(processed_vis, largest_marker, (0, 255, 0), 2)

    def add_no_detection_indicators(self, debug_image, processed_vis):
        """Add indicators when no markers are detected"""
        message = "NO MARKERS DETECTED"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_color = (0, 0, 255)
        font_thickness = 1
        
        text_size = cv2.getTextSize(message, font, font_scale, font_thickness)[0]
        x_pos_debug = int(debug_image.shape[1]/2 - text_size[0]/2)
        x_pos_processed = int(processed_vis.shape[1]/2 - text_size[0]/2)
        
        cv2.putText(debug_image, message, (x_pos_debug, 30), font, font_scale, font_color, font_thickness)
        cv2.putText(processed_vis, message, (x_pos_processed, 60), font, font_scale, font_color, font_thickness)

    def detect_markers_with_multi_threshold(self, image, min_threshold, max_threshold, step):
        """Try multiple thresholds to find the best one for ArUco detection"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        all_detected_markers = []
        best_markers = []
        best_threshold = None
        
        for threshold in range(min_threshold, max_threshold + 1, step):
            _, binary = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY)
            
            kernel = np.ones((3,3), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
            
            corners, ids, _ = cv2.aruco.detectMarkers(
                binary, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None and len(ids) > 0:
                markers_info = self.process_aruco_detection(image, corners, ids, threshold)
                
                if len(markers_info) > len(best_markers):
                    best_markers = markers_info
                    best_threshold = threshold
                
                all_detected_markers.extend(markers_info)
        
        if best_threshold is not None:
            self.get_logger().debug(f"Best threshold: {best_threshold}, detected {len(best_markers)} markers")
        
        return self.filter_unique_markers(all_detected_markers)

    def filter_unique_markers(self, markers):
        """Remove duplicate markers, keeping the largest instance of each ID"""
        unique_markers = {}
        for marker in markers:
            marker_id = marker['id']
            if marker_id not in unique_markers or marker['size'] > unique_markers[marker_id]['size']:
                unique_markers[marker_id] = marker
        
        return list(unique_markers.values())

    def process_aruco_detection(self, image, corners, ids, threshold):
        """Process ArUco detection results into a unified format"""
        marker_info = []
        
        for i in range(len(ids)):
            marker_id = ids[i][0]
            marker_corners = corners[i][0]
            
            perimeter = cv2.arcLength(marker_corners, True)
            
            center_x = int(np.mean([p[0] for p in marker_corners]))
            center_y = int(np.mean([p[1] for p in marker_corners]))
            
            sign_type = self.sign_mapping.get(marker_id, f"ID:{marker_id}")
            
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

    def create_processed_visualization(self, image, detected_markers):
        """Create a visualization of the processed image with markers"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        threshold_value = (self.get_parameter('min_threshold').value + 
                          self.get_parameter('max_threshold').value) // 2
        
        _, binary = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY)
        
        kernel = np.ones((3,3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        processed_vis = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
        
        processed_vis[:,:,0] = binary
        processed_vis[:,:,1] = binary
        processed_vis[:,:,2] = binary
        
        return processed_vis

    def highlight_marker(self, image, marker, color, thickness):
        """Highlight a marker in the image"""
        corners = np.array([marker['corners']], dtype=np.float32)
        cv2.polylines(image, [corners.astype(np.int32)], True, color, thickness)
        
        cv2.circle(image, (marker['center_x'], marker['center_y']), 3, color, -1)
        
        cv2.putText(image, marker['type'], 
                   (marker['center_x'] - 20, marker['center_y'] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

    def add_detection_status(self, debug_image, processed_vis):
        """Add detection status information to debug images"""
        status_text = f"Current: {self.current_detected_sign if self.current_detected_sign else 'None'}"
        
        cv2.putText(debug_image, status_text, (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(processed_vis, status_text, (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    def publish_debug_images(self, debug_image, processed_vis):
        """Publish debug visualization images"""
        if hasattr(self, 'debug_image_publisher') and hasattr(self, 'processed_image_publisher'):
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_publisher.publish(debug_msg)
            
            processed_msg = self.bridge.cv2_to_imgmsg(processed_vis, "bgr8")
            self.processed_image_publisher.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()