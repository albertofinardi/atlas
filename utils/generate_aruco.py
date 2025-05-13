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
    
    # Define total image size
    total_size = 512  # Total image will be 256x256 pixels
    
    # Define border and text area sizes
    border_size = 20
    text_height = 30
    
    # Calculate available width for the marker
    available_width = total_size - (2 * border_size)
    
    # Calculate available height for the marker (subtract text area)
    available_height = total_size - (2 * border_size) - text_height
    
    # Use the smaller dimension to ensure a square marker
    marker_size = min(available_width, available_height)
    
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
        # Generate the square marker
        marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
        marker_img = aruco.drawMarker(dictionary, marker_id, marker_size, marker_img, 1)
        
        # Create a new white image
        bordered_img = np.ones((total_size, total_size), dtype=np.uint8) * 255
        
        # Calculate positions to center the marker horizontally
        marker_start_x = border_size + (available_width - marker_size) // 2
        marker_start_y = border_size
        
        # Place the marker in the centered position
        bordered_img[marker_start_y:marker_start_y+marker_size, 
                     marker_start_x:marker_start_x+marker_size] = marker_img
        
        # Convert to BGR for colored text
        bordered_img_color = cv2.cvtColor(bordered_img, cv2.COLOR_GRAY2BGR)
        
        # Add text with the sign name at the bottom
        text = f"{sign_name}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_thickness = 1
        text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
        
        # Calculate position to center text in the bottom border area
        text_x = (total_size - text_size[0]) // 2
        
        # Calculate the bottom border area and center text vertically within it
        bottom_border_top = border_size + marker_size
        bottom_border_center = bottom_border_top + text_height // 2
        
        # Adjust for text baseline to properly center it
        text_y = bottom_border_center + text_size[1] // 2
        
        cv2.putText(bordered_img_color, text, (text_x, text_y),
                    font, font_scale, (0, 0, 0), font_thickness)
        
        # Save the marker with border and text
        filename = f"aruco_markers/aruco_{marker_id}_{sign_name}.png"
        cv2.imwrite(filename, bordered_img_color)
        print(f"Saved marker: {filename}")

if __name__ == "__main__":
    generate_aruco_markers()
    print("All markers generated successfully!")