import cv2
import cv2.aruco as aruco
import numpy as np
import os

def generate_aruco_markers():
    # Create directory for markers if it doesn't exist
    if not os.path.exists('aruco_markers'):
        os.makedirs('aruco_markers')
    
    # Define dictionary
    dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
    
    # Define marker size (pixels)
    marker_size = 256  # 256x256 pixels
    
    # Define mapping
    sign_mapping = {
        0: "LIGHT_RED",
        1: "LIGHT_YELLOW",
        2: "LIGHT_GREEN",
        3: "SPEED_30",
        4: "SPEED_50"
    }
    
    # Generate and save each marker
    for marker_id, sign_name in sign_mapping.items():
        # Generate the marker
        marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
        marker_img = aruco.drawMarker(dictionary, marker_id, marker_size, marker_img, 1)
        
        # Save the marker - pure black and white
        filename = f"aruco_markers/aruco_{marker_id}_{sign_name}.png"
        cv2.imwrite(filename, marker_img)
        print(f"Saved marker: {filename}")

if __name__ == "__main__":
    generate_aruco_markers()
    print("All markers generated successfully!")