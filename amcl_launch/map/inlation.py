import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

# Cargar mapa como escala de grises
map_img = cv2.imread("mapa.pgm", cv2.IMREAD_GRAYSCALE)

# Umbral para determinar obstáculos (puede depender de tu threshold YAML)
_, binary = cv2.threshold(map_img, 250, 255, cv2.THRESH_BINARY_INV)

# Aplicar inflación (distance transform)
dist_transform = cv2.distanceTransform(binary, cv2.DIST_L2, 5)

inflation_radius_px = 0.3/0.05  # ajusta según tamaño del robot en píxeles
inflated = np.clip(inflation_radius_px - dist_transform, 0, inflation_radius_px)
costmap = (inflated / inflation_radius_px) * 100

plt.imshow(costmap, cmap='hot')
plt.title("Visualización de Costmap Inflado")
plt.colorbar()
plt.show()
