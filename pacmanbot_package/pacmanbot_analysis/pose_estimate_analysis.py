import os
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# --------------------------------------------------
# FILE PATHS
# --------------------------------------------------
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(BASE_PATH, "data",)

# Change this to your actual filename
FILE_NAME = "amcl_pose_trial.csv"

file_path = os.path.join(DATA_PATH, FILE_NAME)

if not os.path.exists(file_path):
    raise FileNotFoundError(f"Missing file: {file_path}")


# --------------------------------------------------
# LOAD CSV
# --------------------------------------------------
df = pd.read_csv(file_path)

print("\nColumns found in CSV:")
print(df.columns.tolist())


# --------------------------------------------------
# HELPER: quaternion -> yaw
# --------------------------------------------------
def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# --------------------------------------------------
# PARSE COLUMNS
# Reuse same logic as your old logger format
# --------------------------------------------------
theta_vals = None

if "x" in df.columns and "y" in df.columns:
    x_vals = df["x"].to_numpy()
    y_vals = df["y"].to_numpy()

    if "theta" in df.columns:
        theta_vals = df["theta"].to_numpy()

elif "pose.pose.position.x" in df.columns and "pose.pose.position.y" in df.columns:
    x_vals = df["pose.pose.position.x"].to_numpy()
    y_vals = df["pose.pose.position.y"].to_numpy()

    quat_cols = [
        "pose.pose.orientation.x",
        "pose.pose.orientation.y",
        "pose.pose.orientation.z",
        "pose.pose.orientation.w",
    ]

    if all(col in df.columns for col in quat_cols):
        qx = df["pose.pose.orientation.x"].to_numpy()
        qy = df["pose.pose.orientation.y"].to_numpy()
        qz = df["pose.pose.orientation.z"].to_numpy()
        qw = df["pose.pose.orientation.w"].to_numpy()

        theta_vals = np.array([
            quaternion_to_yaw(qx[i], qy[i], qz[i], qw[i])
            for i in range(len(df))
        ])
else:
    print("\nCSV columns were:")
    print(df.columns.tolist())
    raise ValueError("Unknown CSV format. Update the parser to match your logger output.")


# --------------------------------------------------
# BASIC STATISTICS
# --------------------------------------------------
mean_x = np.mean(x_vals)
mean_y = np.mean(y_vals)

std_x = np.std(x_vals)
std_y = np.std(y_vals)

radial_error = np.sqrt((x_vals - mean_x) ** 2 + (y_vals - mean_y) ** 2)
mean_radial_error = np.mean(radial_error)
max_radial_error = np.max(radial_error)

print("\n===== AMCL 2D POSE ESTIMATE ANALYSIS =====")
print(f"Samples: {len(x_vals)}")
print(f"Mean X: {mean_x:.4f} m")
print(f"Mean Y: {mean_y:.4f} m")
print(f"Std Dev X: {std_x:.4f} m")
print(f"Std Dev Y: {std_y:.4f} m")
print(f"Mean radial spread: {mean_radial_error:.4f} m")
print(f"Max radial spread: {max_radial_error:.4f} m")

if theta_vals is not None:
    mean_theta = np.mean(theta_vals)
    std_theta = np.std(theta_vals)

    print(f"Mean Theta: {mean_theta:.4f} rad ({np.degrees(mean_theta):.2f} deg)")
    print(f"Std Dev Theta: {std_theta:.4f} rad ({np.degrees(std_theta):.2f} deg)")


# --------------------------------------------------
# PLOT 1: XY SCATTER
# --------------------------------------------------
plt.figure(figsize=(8, 6))
plt.scatter(x_vals, y_vals, s=20, alpha=0.7, label="AMCL estimates")
plt.scatter(mean_x, mean_y, marker="x", s=100, label="Mean estimate")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("AMCL Estimates from Repeated 2D Pose Estimate Updates")
plt.axis("equal")
plt.grid(True)
plt.legend()


# --------------------------------------------------
# PLOT 2: RADIAL ERROR VS SAMPLE INDEX
# --------------------------------------------------
plt.figure(figsize=(8, 5))
plt.plot(radial_error)
plt.xlabel("Sample Index")
plt.ylabel("Distance from Mean (m)")
plt.title("Radial Spread of AMCL Estimates")
plt.grid(True)


# --------------------------------------------------
# PLOT 3: X AND Y OVER TIME
# --------------------------------------------------
plt.figure(figsize=(8, 5))
plt.plot(x_vals, label="X")
plt.plot(y_vals, label="Y")
plt.xlabel("Sample Index")
plt.ylabel("Position (m)")
plt.title("AMCL Position Estimates Over Samples")
plt.grid(True)
plt.legend()


# --------------------------------------------------
# PLOT 4: THETA OVER TIME
# --------------------------------------------------
if theta_vals is not None:
    plt.figure(figsize=(8, 5))
    plt.plot(np.degrees(theta_vals))
    plt.xlabel("Sample Index")
    plt.ylabel("Theta (deg)")
    plt.title("AMCL Heading Estimates Over Samples")
    plt.grid(True)

plt.tight_layout()
plt.show()