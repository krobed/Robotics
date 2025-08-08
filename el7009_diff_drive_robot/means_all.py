import pandas as pd
import matplotlib.pyplot as plt
import os

# Load CSV
df = pd.read_csv(f"{os.path.dirname(__file__)}/data.csv")

# Column with distances
distances = df["Distance to Goal"]

# Mean and std of all data
mean_distance = distances.mean()
std_distance = distances.std()

print(f"Mean distance to goal: {mean_distance:.3f}")
print(f"Std distance to goal: {std_distance:.3f}")

# --- Plot ---
plt.figure(figsize=(6, 5))

# Mean ± std as a point with error bar
plt.errorbar(
    0, mean_distance,
    yerr=std_distance,
    fmt="o",
    capsize=5,
    color="blue",
    ecolor="black",
    label="Mean ± Std"
)

# All individual values (jittered horizontally for visibility)
plt.scatter(
    [0.05] * len(distances),  # small offset from the mean
    distances,
    color="orange",
    alpha=0.6,
    s=20,
    label="Individual values"
)

plt.xticks([0, 0.05], ["Mean", "Values"])
plt.ylabel("Distance to Goal (m)")
plt.title("Distance to Goal (All Data)")
plt.legend()
plt.tight_layout()
plt.show()
