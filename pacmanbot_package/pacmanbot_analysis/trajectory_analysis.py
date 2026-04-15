import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# PATH SETUP 
# -----------------------------
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(BASE_PATH, "data")

files = [
    "amcl_t1.csv",
    "amcl_t2.csv",
    "amcl_t3.csv",
    "amcl_t4.csv"
]

trials = {}

# -----------------------------
# LOAD DATA
# -----------------------------
for f in files:
    path = os.path.join(DATA_PATH, f)

    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing file: {path}")

    df = pd.read_csv(path)

    # Try common ROS formats
    if "x" in df.columns and "y" in df.columns:
        x = df["x"].values
        y = df["y"].values

    elif "pose.pose.position.x" in df.columns:
        x = df["pose.pose.position.x"].values
        y = df["pose.pose.position.y"].values

    else:
        print(f"\nColumns found in {f}:")
        print(df.columns)
        raise ValueError(f"Unknown column format in {f}")

    trials[f] = {
        "x": x,
        "y": y
    }

# -----------------------------
# ANALYSIS
# -----------------------------
print("\n===== AMCL ANALYSIS =====\n")

stats = {}

for name, data in trials.items():
    x = data["x"]
    y = data["y"]

    mean_x = np.mean(x)
    mean_y = np.mean(y)

    std_x = np.std(x)
    std_y = np.std(y)

    stats[name] = {
        "mean_x": mean_x,
        "mean_y": mean_y,
        "std_x": std_x,
        "std_y": std_y
    }

    print(f"{name}:")
    print(f"  Mean Position: ({mean_x:.4f}, {mean_y:.4f})")
    print(f"  Std Dev:      (x: {std_x:.4f}, y: {std_y:.4f})\n")

# -----------------------------
# TRAJECTORY PLOT
# -----------------------------
plt.figure()

for name, data in trials.items():
    plt.plot(data["x"], data["y"], label=name)

plt.title("AMCL Trajectories (All Trials)")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend()
plt.axis("equal")
plt.grid()

# -----------------------------
# SCATTER PLOT (NOISE)
# -----------------------------
plt.figure()

for name, data in trials.items():
    plt.scatter(data["x"], data["y"], s=5, label=name)

plt.title("AMCL Scatter (Noise)")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend()
plt.axis("equal")
plt.grid()

# -----------------------------
# FINAL POSITION VARIATION
# -----------------------------
endpoints = []

for data in trials.values():
    endpoints.append([data["x"][-1], data["y"][-1]])

endpoints = np.array(endpoints)

pairwise_dist = []

for i in range(len(endpoints)):
    for j in range(i+1, len(endpoints)):
        dist = np.linalg.norm(endpoints[i] - endpoints[j])
        pairwise_dist.append(dist)

print("Average final position difference:", np.mean(pairwise_dist))

plt.show()