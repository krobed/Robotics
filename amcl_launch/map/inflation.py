import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

# Cargar mapa como escala de grises
path = os.path.dirname(__file__)
map_img = cv2.imread(f"{path}/mapa.pgm", cv2.IMREAD_GRAYSCALE)

# Umbral para determinar obstáculos (puede depender de tu threshold YAML)
_, binary = cv2.threshold(map_img, 250, 255, cv2.THRESH_BINARY_INV)

# Aplicar inflación (distance transform)
dist_transform = cv2.distanceTransform(binary, cv2.DIST_L2, cv2.DIST_MASK_5)

inflation_radius_px = 5 # ajusta según tamaño del robot en píxeles
inflated = np.clip(inflation_radius_px - dist_transform, 0, inflation_radius_px)
costmap = (inflated / inflation_radius_px) * 100

plt.imshow(costmap, cmap='gray')
plt.title("Visualización de Costmap Inflado")
plt.colorbar()
plt.show()
