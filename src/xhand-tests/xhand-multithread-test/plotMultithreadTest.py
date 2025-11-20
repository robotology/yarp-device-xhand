#!/usr/bin/env python3
"""
Load a CSV file (first row header), print mean/std per column and plot each column on a separate row.
Usage:
  python3 plotTest.py /path/to/file.csv
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

def main():
    p = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("xhand-multithread-test.csv")
    if not p.exists():
        print(f"File not found: {p}", file=sys.stderr)
        sys.exit(1)

    with p.open('r') as f:
        header = [h.strip() for h in f.readline().strip().split(',') if h.strip()]
        data = np.loadtxt(f, delimiter=',')
    if data.ndim == 1:
        data = data.reshape(1, -1)

    means = data.mean(axis=0)
    stds = data.std(axis=0, ddof=0)

    samples = np.arange(1, data.shape[0] + 1)
    ncols = data.shape[1]

    fig, axes = plt.subplots(ncols, 1, figsize=(8, 2.6 * max(1, ncols)), sharex=True)
    if ncols == 1:
        axes = [axes]

    time_unit = "ns"

    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for i, ax in enumerate(axes):
        series = data[:, i]
        mean = means[i]
        std = stds[i]
        ax.plot(samples, series, marker='o', linestyle='none', color=colors[i % len(colors)])
        ax.axhline(mean, color='k', linestyle='--', linewidth=0.9, label=f"mean={mean:.3f} {time_unit}")
        ax.fill_between(samples, mean - std, mean + std, color=colors[i % len(colors)], alpha=0.15,
                        label=f"±1σ={std:.3f} {time_unit}")
        ax.set_ylabel(f"[{time_unit}]")
        ax.set_title(header[i])
        ax.grid(True, linestyle='--', alpha=0.4)
        ax.legend(loc='upper right', fontsize='small')
    axes[-1].set_xlabel("Sample")

    plt.suptitle(p.name)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    out = p.with_suffix('.png')
    plt.savefig(out, dpi=150)
    print(f"Saved plot to: {out}")

    # Histograms
    fig, axes = plt.subplots(ncols, 1, figsize=(8, 2.6 * max(1, ncols)))
    if ncols == 1:
        axes = [axes]

    for i, ax in enumerate(axes):
        series = data[:, i]
        mean = means[i]
        std = stds[i]
        bins = series.size // 100 if series.size // 100 > 10 else 10

        counts, bin_edges = np.histogram(series, bins=bins)
        max_bin_index = np.argmax(counts)
        mode = 0.5 * (bin_edges[max_bin_index] + bin_edges[max_bin_index + 1])
        mode_perc = (counts[max_bin_index] / series.size) * 100 if series.size > 0 else 0

        ax.set_ylim(0, counts[max_bin_index] * 1.1)
        ax.hist(series, bins=bins, color=colors[i % len(colors)], alpha=0.7)
        stats_table = [
            ["mean", f"{mean:.3f} {time_unit}"],
            ["std",  f"{std:.3f} {time_unit}"],
            ["mode", f"{mode:.3f} {time_unit}"],
            ["mode %", f"{mode_perc:.3f} %"],
        ]

        table = ax.table(
            cellText=stats_table,
            loc='upper right',
            cellLoc='left',
            bbox=[0.65, 0.70, 0.30, 0.25]
        )
        table.auto_set_font_size(False)
        table.set_fontsize(8)

        ax.set_xlabel(f"[{time_unit}]")
        ax.set_title(f"Histogram of {header[i]} - bin width: {(bin_edges[1]-bin_edges[0]):.3f} {time_unit}  - bin num/samples: {bins}/{series.size}")
        ax.grid(True, linestyle='--', alpha=0.4)

        # For each bin, print range and count and percentage
        print(f"Histogram data for {header[i]}:")
        print(f"mean [{time_unit}] ={means[i]:.6f}  std [{time_unit}]={stds[i]:.6f}  mode [{time_unit}]={mode:.6f} - ({mode_perc:.3f}%)")
        # total_counts = series.size
        # for j in range(len(counts)):
        #     bin_start = bin_edges[j]
        #     bin_end = bin_edges[j + 1]
        #     count = counts[j]
        #     percentage = (count / total_counts) * 100 if total_counts > 0 else 0
        #     if percentage > 0:
        #         print(f"  Bin {j}: [{bin_start:.6f}, {bin_end:.6f}) -> Count: {count}, Percentage: {percentage:.3f}%")
        # print("")


    plt.suptitle(f"Histograms - {p.name}")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_hist = p.name.replace('.csv', '-hist.png')
    plt.savefig(out_hist, dpi=150)
    print(f"Saved histogram plot to: {out_hist}")
    plt.show()

if __name__ == "__main__":
    main()