# Author: Meissam Safi(s224298)

#Importing libraries
import cv2

# Function for drawing the grid 
def draw_grid(frame, grid_size=(10, 10), color=(0, 255, 0), thickness=1):
    
    height, width = frame.shape[:2]
    cell_width, cell_height = grid_size

    # Draw vertical lines
    for x in range(0, width, cell_width):
        cv2.line(frame, (x, 0), (x, height), color, thickness)

    # Draw horizontal lines
    for y in range(0, height, cell_height):
        cv2.line(frame, (0, y), (width, y), color, thickness)

    return frame
