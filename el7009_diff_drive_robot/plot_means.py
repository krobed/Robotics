import pandas as pd
import matplotlib.pyplot as plt
import os
# Load CSV
df = pd.read_csv(f"{os.path.dirname(__file__)}/data.csv")

# Create a "pair" column
df["pair"] = list(zip(
    df["init"],
    df["goal"]
))

# Group: mean, std, and list of values
stats = df.groupby("pair")["Distance to Goal"].agg(
    mean="mean",
    std="std",
    values=lambda x: list(x)
).reset_index()

# --- Plot mean ± std ---
plt.figure(figsize=(10, 5))
plt.errorbar(
    range(len(stats)),
    stats["mean"],
    yerr=stats["std"],
    fmt="o",
    capsize=5,
    color="blue",
    ecolor="black",
    label="Mean ± Std"
)

# Plot individual values (slightly jittered for visibility)
for i, vals in enumerate(stats["values"]):
    plt.scatter(
        [i] * len(vals),  # same x position
        vals,
        color="orange",
        alpha=0.6,
        s=20,
        label="Individual values" if i == 0 else None  # only label first for legend
    )

plt.xticks(range(len(stats)), stats["pair"], rotation=45, ha="right")
plt.xlabel("Initial Position → Goal Position")
plt.ylabel("Plan Distance (m)")
plt.title("Plan Distance by Position Pair (Mean ± Std & Individual Values)")
plt.legend()
plt.tight_layout()
plt.show()
