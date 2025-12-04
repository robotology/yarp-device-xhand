#!/usr/bin/env python3
"""
Load a CSV file (first row data_header), print mean/std per column and plot each column on a separate row.
Usage:
  python3 plotTest.py /path/to/file.csv
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import pickle

def main():
    p = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("xhand-multithread-test.csv")
    if not p.exists():
        print(f"File not found: {p}", file=sys.stderr)
        sys.exit(1)

    with p.open('r') as f:
        data_header = [h.strip() for h in f.readline().strip().split(',') if h.strip()]
        test_data = np.loadtxt(f, delimiter=',')
    if test_data.ndim == 1:
        test_data = test_data.reshape(1, -1)

    # Select last 2 columns (data)
    data = test_data[:, 2:]
    data_header = data_header[2:]
    # Select first 2 columns (timestamps)
    data_timestamps = test_data[:, :2]    
    data_timestamps = data_timestamps - data_timestamps.min()

    means = data.mean(axis=0)
    stds = data.std(axis=0, ddof=0)

    # Use tha same bin num and edges for all histograms
    all_series = data.flatten()
    bins = all_series.size // 100 if all_series.size // 100 > 10 else 10
    _, bin_edges = np.histogram(all_series, bins=bins)
    hystogram_y_lim = 0
    counts = np.zeros((bins, data.shape[1]))
    for i in range(data.shape[1]):
        series = data[:, i]
        counts[:, i], _ = np.histogram(series, bins=bin_edges)
        max_bin_index = np.argmax(counts[:, i])
        if counts[:, i][max_bin_index] * 1.1 > hystogram_y_lim:
            hystogram_y_lim = counts[:, i][max_bin_index] * 1.1


    samples = data.shape[0]
    ncols = data.shape[1]
    time_unit = "ns"

    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    # Histograms
    fig, axes = plt.subplots(ncols, 1, figsize=(8, 2.6 * max(1, ncols)))
    if ncols == 1:
        axes = [axes]

    for i, ax in enumerate(axes):
        series = data[:, i]
        max_val = max(series)
        mean = means[i]
        std = stds[i]
        count = counts[:, i]

        max_bin_index = np.argmax(count)
        mode = 0.5 * (bin_edges[max_bin_index] + bin_edges[max_bin_index + 1])
        mode_perc = (count[max_bin_index] / series.size) * 100 if series.size > 0 else 0

        ax.set_ylim(0, hystogram_y_lim)
        ax.hist(series, bins=bin_edges, color=colors[i % len(colors)], alpha=0.7)
        ax.set_xscale('log')
        stats_table = [
            ["max", f"{max_val:.3f} {time_unit}"],
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

        ax.set_xlabel(f"cycle time [{time_unit}]")
        ax.set_ylabel(f"counts [{time_unit}]")
        ax.set_title(f"{data_header[i]}: counts vs cycle time")
        ax.grid(True, linestyle='--', alpha=0.4)

        # For each bin, print range and count and percentage
        print(f"Histogram data for {data_header[i]}:")
        print(f" max [{time_unit}] ={max_val:.6f}  mean [{time_unit}]={means[i]:.6f}  std [{time_unit}]={stds[i]:.6f}  mode [{time_unit}]={mode:.6f} - ({mode_perc:.3f}%)")

    plt.suptitle(f"Histograms - {p.name} \n bin width: {(bin_edges[1]-bin_edges[0]):.3f} {time_unit}  - bin num/samples: {bins}/{samples}")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_hist = p.name.replace('.csv', '-hist.png')
    plt.savefig(out_hist, dpi=150)

    print(f"Saved histogram plot to: {out_hist}")
    with open(out_hist.replace('.png', '.pkl'), "wb") as f:
        pickle.dump(fig, f)

    #Plot data vs timestamps on the same plot
    fig, ax = plt.subplots(figsize=(8, 2.6))
    for i in range(ncols):
        series = data[:, i]
        timestamps = data_timestamps[:, i]
        ax.plot(timestamps, series, marker='o', linestyle='none', label=data_header[i], color=colors[i % len(colors)])
    ax.set_ylabel(f"Cycle time [{time_unit}]")
    ax.set_title(f"Cycle time vs Timestamps")
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.set_xlabel(f"Timestamps [{time_unit}]")
    ax.legend(loc='upper right', fontsize='small')
    plt.suptitle(f"Timestamps - {p.name}")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_ts_all = p.name.replace('.csv', '-ct-ts.png')
    plt.savefig(out_ts_all, dpi=150)
    print(f"Saved timestamp (all data) plot to: {out_ts_all}")
    with open(out_ts_all.replace('.png', '.pkl'), "wb") as f:
        pickle.dump(fig, f)

    plt.show()

if __name__ == "__main__":
    main()