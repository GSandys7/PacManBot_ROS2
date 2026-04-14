import cv2

src = '/home/eva/ros2_ws/src/PacManBot_ROS2/pacmanbot_package/maps/map_01.pgm'
dst = '/home/eva/ros2_ws/src/PacManBot_ROS2/pacmanbot_package/maps/map_01_clean.pgm'

img = cv2.imread(src, cv2.IMREAD_GRAYSCALE)

# FORCE EVERYTHING into strict binary
binary = (img > 240).astype('uint8') * 255

cv2.imwrite(dst, binary)

print("Saved binary map")

# sanity check
vals = set(binary.flatten())
print("Unique values:", vals)
