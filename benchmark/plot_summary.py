#!/usr/bin/env python3

import csv
from pathlib import Path

import matplotlib.pyplot as plt


def main() -> None:
    benchmark_directory = Path(__file__).resolve().parent
    with (benchmark_directory / "summary.csv").open(encoding="utf-8") as summary_file:
        rows = list(csv.DictReader(summary_file))

    labels = [row["implementation"] for row in rows]
    colors = ["#727272", "#0072B2"]
    metrics = [
        ("total_mean_ms", "Mean scan time", "ms / scan", False),
        ("cpu_core_percent", "Process CPU", "% of one core", False),
        ("successful_only_ape_rmse_m", "Translation APE RMSE", "m", True),
    ]

    figure, axes = plt.subplots(1, 3, figsize=(12.8, 4.2))
    for axis, (column, title, unit, logarithmic) in zip(axes, metrics):
        values = [float(row[column]) for row in rows]
        bars = axis.bar(labels, values, color=colors, width=0.62)
        axis.set_title(title, fontweight="bold")
        axis.set_ylabel(unit)
        axis.grid(axis="y", alpha=0.25)
        axis.tick_params(axis="x", labelrotation=12)
        if logarithmic:
            axis.set_yscale("log")
        for bar, value in zip(bars, values):
            axis.text(
                bar.get_x() + bar.get_width() / 2.0,
                value * (1.12 if logarithmic else 1.02),
                f"{value:.3f}",
                ha="center",
                va="bottom",
                fontsize=9,
            )

    figure.suptitle("FAST-LIO2 Original vs. Surfel FAST-LIO2 Eigen (12 sequences × 3 runs)")
    figure.tight_layout()
    figure.savefig(
        benchmark_directory.parent / "assets" / "benchmark_overview.svg",
        bbox_inches="tight",
    )


if __name__ == "__main__":
    main()
