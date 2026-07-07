# ------------------------------------------------------------------
# PERFORMANCE ANALYSIS
#
# This script analyzes the performance metrics collected during the
# benchmark experiments and generates summary statistics together
# with publication-quality plots.
#
# The analysis includes:
# - Distribution of planning time and makespan
# - Mean and standard deviation comparison
# - Scalability with respect to problem size
# - Performance variation across benchmark instances
# ------------------------------------------------------------------

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import ScalarFormatter

# ------------------------------------------------------------------
# General plotting configuration.
#
# The title map associates each metric stored in the CSV file with
# a human-readable label used in figures and axis descriptions.
# ------------------------------------------------------------------

CSV_FILE = "metrics.csv"

title_map = {
    "time": "Planning time (ms)",
    "makespan": "Makespan (timesteps)"
}

# ------------------------------------------------------------------
# Loads the experiment results.
#
# The CSV file contains one row per recorded metric, including the
# algorithm name, benchmark instance, number of boxes, metric type,
# and measured value.
# ------------------------------------------------------------------

df = pd.read_csv(CSV_FILE)
df["value"] = pd.to_numeric(df["value"])

# ------------------------------------------------------------------
# Aggregates the experimental measurements.
#
# Planning time is averaged over the three executions performed for
# each benchmark instance in order to reduce execution variability.
#
# Makespan is deterministic for a given solution and therefore only
# requires a single recorded value.
# ------------------------------------------------------------------

df_time = (
    df[df["metric"] == "time"]
    .groupby(["algorithm", "box", "example"], as_index=False)["value"]
    .mean()
)
df_time["metric"] = "time"

df_makespan = df[df["metric"] == "makespan"].copy()

df_agg = pd.concat([df_time, df_makespan], ignore_index=True)

# ------------------------------------------------------------------
# Defines a consistent color palette for all algorithms so that the
# same algorithm is represented with the same color in every figure.
# ------------------------------------------------------------------

plt.rcParams["figure.figsize"] = (10, 6)

algorithms = df_agg["algorithm"].unique()
colors = plt.cm.Set2(range(len(algorithms)))
color_map = dict(zip(algorithms, colors))

# ------------------------------------------------------------------
# 1. Distribution analysis (Boxplot)
#
# Generates one boxplot per metric to visualize the distribution,
# spread, median, and potential outliers for each algorithm.
# ------------------------------------------------------------------

for metric in df_agg["metric"].unique():

    plt.figure()

    data = [
        df_agg[(df_agg["metric"] == metric) & (df_agg["algorithm"] == alg)]["value"]
        for alg in algorithms
    ]

    plt.boxplot(data, tick_labels=algorithms)

    plt.title(f"{title_map[metric]} - Distribution")
    plt.ylabel(title_map[metric])

    if metric == "time":

        formatter = ScalarFormatter()
        formatter.set_scientific(False)
        formatter.set_useOffset(False)

        plt.gca().yaxis.set_major_formatter(formatter)

    plt.minorticks_on()

    if metric == "time":
      plt.grid(
          True,
          which="both",
          axis="y",
          linestyle="--",
          alpha=0.6
      )
    if metric == "makespan":
      plt.grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    plt.savefig(f"boxplot_{metric}.png", dpi=300)
    plt.show()

# ------------------------------------------------------------------
# 2. Mean performance comparison (Mean ± Std)
#
# Computes the mean, standard deviation, and coefficient of variation
# for every algorithm and metric. The resulting bar plots display the
# average value together with error bars representing one standard
# deviation.
# ------------------------------------------------------------------

summary = df_agg.groupby(["algorithm", "metric"])["value"].agg(["mean", "std"]).reset_index()
summary["cv"] = summary["std"] / summary["mean"]

for metric in summary["metric"].unique():

    plt.figure()

    subset = summary[summary["metric"] == metric]

    x = np.arange(len(subset))

    plt.bar(
        x,
        subset["mean"],
        yerr=subset["std"],
        capsize=5,
        color=[color_map[a] for a in subset["algorithm"]]
    )

    plt.xticks(x, subset["algorithm"])
    plt.title(f"{title_map[metric]} - Mean ± Std")
    plt.ylabel(title_map[metric])

    if metric == "time":
        plt.yscale("log")

        formatter = ScalarFormatter()
        formatter.set_scientific(False)
        formatter.set_useOffset(False)

        plt.gca().yaxis.set_major_formatter(formatter)

    plt.minorticks_on()

    if metric == "time":
      plt.grid(
          True,
          which="both",
          axis="y",
          linestyle="--",
          alpha=0.6
      )

    if metric == "makespan":
      plt.grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    plt.savefig(f"mean_std_{metric}.png", dpi=300)
    plt.show()

# ------------------------------------------------------------------
# 3. Scalability analysis (per box quantity)
#
# Evaluates how each metric evolves as the problem size increases.
# The mean value is computed for each number of boxes and plotted as
# a performance curve for every algorithm.
# ------------------------------------------------------------------

scale = df_agg.groupby(["algorithm", "metric", "box"])["value"].mean().reset_index()

for metric in scale["metric"].unique():

    plt.figure()

    subset = scale[scale["metric"] == metric]

    for alg in algorithms:

        data_alg = subset[subset["algorithm"] == alg]

        plt.plot(
            data_alg["box"],
            data_alg["value"],
            marker="o",
            label=alg,
            color=color_map[alg]
        )

    plt.title(f"{title_map[metric]} vs Problem Size")
    plt.xlabel("Number of Boxes")
    plt.ylabel(title_map[metric])
    plt.legend()

    if metric == "time":
        plt.yscale("log")

        formatter = ScalarFormatter()
        formatter.set_scientific(False)
        formatter.set_useOffset(False)

        plt.gca().yaxis.set_major_formatter(formatter)

    if metric == "time":
      plt.grid(
          True,
          which="both",
          axis="y",
          linestyle="--",
          alpha=0.6
      )

    if metric == "makespan":
      plt.grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    plt.savefig(f"scaling_{metric}.png", dpi=300)
    plt.show()

# ------------------------------------------------------------------
# 4. Instance variability (example-level analysis)
#
# Evaluates how algorithm performance varies across different
# benchmark instances while keeping the problem size fixed.
#
# This helps identify whether particular examples are consistently
# more difficult than others.
# ------------------------------------------------------------------

BOX_QNT=10

trend = (
    df_agg[df_agg["box"] == BOX_QNT]
    .groupby(["algorithm", "metric", "example"])["value"]
    .mean()
    .reset_index()
)

for metric in trend["metric"].unique():

    plt.figure()

    subset = trend[trend["metric"] == metric]

    for alg in algorithms:

        data_alg = subset[subset["algorithm"] == alg]

        plt.plot(
            data_alg["example"],
            data_alg["value"],
            marker="o",
            label=alg,
            color=color_map[alg]
        )

    plt.title(f"{title_map[metric]} vs Example (10 Boxes)")
    plt.xlabel("Example ID")
    plt.ylabel(title_map[metric])
    plt.legend()

    if metric == "time":
        plt.yscale("log")

        formatter = ScalarFormatter()
        formatter.set_scientific(False)
        formatter.set_useOffset(False)

        plt.gca().yaxis.set_major_formatter(formatter)

    if metric == "time":
      plt.grid(
          True,
          which="both",
          axis="y",
          linestyle="--",
          alpha=0.6
      )

    if metric == "makespan":
      plt.grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    plt.savefig(f"trend_{metric}.png", dpi=300)
    plt.show()

# ------------------------------------------------------------------
# Computes the complete statistical summary for every algorithm and
# metric.
#
# The generated table includes:
# - Mean
# - Median
# - Standard deviation
# - Minimum
# - Maximum
# - Coefficient of variation
#
# The summary is printed to the console and exported as a CSV file.
# ------------------------------------------------------------------

stats = (
    df_agg.groupby(["algorithm", "metric"])["value"]
    .agg(["mean", "median", "std", "min", "max"])
    .rename_axis(index=["Algorithms", "Metric"])
)

stats["cv"] = stats["std"] / stats["mean"]

print(stats)

stats.to_csv("stats.csv")

print("Analysis complete.")