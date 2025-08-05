import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

# Load the PGM map (values: -1, 0, 100)
path = os.path.dirname(__file__)
map_img = cv2.imread(f"{path}/mapa.pgm", cv2.IMREAD_UNCHANGED).astype(np.int16)  # Keep -1 intact
plt.imshow(map_img)
plt.show()
# Create a binary mask of the occupied space
occupied_mask = (map_img == 0).astype(np.uint8)

# Define a circular structuring element of radius 5
radius = 5
kernel_size = 2 * radius + 1
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

# Dilate the occupied regions
dilated_occupied = cv2.dilate(occupied_mask, kernel, iterations=1)

# Create a new map: start from the original
dilated_map = map_img.copy()

# Set newly occupied cells to 100 (but leave unknowns (-1) untouched)
dilated_map[(dilated_occupied == 1) & (map_img == 254)]=205 # Only fill free cells

# Save the result as PGM again (convert to uint8 first)
cv2.imwrite(f"{path}/map_dilated.pgm", dilated_map)
plt.imshow(dilated_map)
plt.show()