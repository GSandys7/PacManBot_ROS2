import yaml
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# =========================
# CONFIG
# =========================
yaml_file = "/maps/map_name.yaml"   # <-- change to your YAML file
num_points = 10          # how many random points to test

# =========================
# LOAD YAML
# =========================
with open(yaml_file, 'r') as f:
    map_metadata = yaml.safe_load(f)

image_path = map_metadata["image"]
resolution = map_metadata["resolution"]
origin = map_metadata["origin"]  # [x, y, yaw]

origin_x, origin_y, origin_theta = origin

print("Map Info:")
print(f"  Resolution: {resolution} m/cell")
print(f"  Origin: {origin}")

# =========================
# LOAD PGM IMAGE
# =========================
img = Image.open(image_path)
map_array = np.array(img)

# Flip vertically to match ROS coordinate system
map_array = np.flipud(map_array)

height, width = map_array.shape
print(f"  Map size: {width} x {height} cells")

# =========================
# GRID -> WORLD FUNCTION
# =========================
def grid_to_world(i, j):
    x = origin_x + (i + 0.5) * resolution
    y = origin_y + (j + 0.5) * resolution
    return x, y

# =========================
# SAMPLE RANDOM FREE CELLS
# =========================
free_mask = map_array > 200  # white-ish = free

free_indices = np.argwhere(free_mask)

# randomly sample some points
sample_indices = free_indices[np.random.choice(len(free_indices), num_points, replace=False)]

grid_points = []
world_points = []

for (j, i) in sample_indices:  # note: numpy uses (row, col) = (y, x)
    grid_points.append((i, j))
    world_points.append(grid_to_world(i, j))

# =========================
# PRINT RESULTS
# =========================
print("\nSample Conversions:")
for k in range(len(grid_points)):
    gi, gj = grid_points[k]
    wx, wy = world_points[k]
    print(f"Grid ({gi}, {gj}) -> World ({wx:.2f}, {wy:.2f})")

# =========================
# PLOTTING
# =========================
plt.figure(figsize=(8, 8))
plt.imshow(map_array, cmap='gray')

# Plot grid points
for (i, j) in grid_points:
    plt.scatter(i, j, c='red', s=30)

plt.title("Grid Points on Map")
plt.xlabel("Grid X")
plt.ylabel("Grid Y")

plt.gca().invert_yaxis()
plt.show()

# =========================
# WORLD PLOT (OPTIONAL)
# =========================
wx_list = [p[0] for p in world_points]
wy_list = [p[1] for p in world_points]

plt.figure(figsize=(6, 6))
plt.scatter(wx_list, wy_list, c='blue')

plt.title("Same Points in World Coordinates")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.grid()
plt.axis('equal')
plt.show()