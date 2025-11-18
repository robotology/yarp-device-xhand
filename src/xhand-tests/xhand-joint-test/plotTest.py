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
    p = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("xhand-joint-test-read-first.csv")
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

    for name, m, s in zip(header, means, stds):
        print(f"{name}: mean={m:.6f}  std={s:.6f}")

    samples = np.arange(1, data.shape[0] + 1)
    ncols = data.shape[1]

    fig, axes = plt.subplots(ncols, 1, figsize=(8, 2.6 * max(1, ncols)), sharex=True)
    if ncols == 1:
        axes = [axes]

    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for i, ax in enumerate(axes):
        series = data[:, i]
        mean = means[i]
        std = stds[i]
        ax.plot(samples, series, marker='o', linestyle='-', color=colors[i % len(colors)])
        ax.axhline(mean, color='k', linestyle='--', linewidth=0.9, label=f"mean={mean:.3f} ms")
        ax.fill_between(samples, mean - std, mean + std, color=colors[i % len(colors)], alpha=0.15,
                        label=f"±1σ={std:.3f} ms")
        ax.set_ylabel("[ms]")
        ax.set_title(header[i])
        ax.grid(True, linestyle='--', alpha=0.4)
        ax.legend(loc='upper right', fontsize='small')
    axes[-1].set_xlabel("Sample")

    plt.suptitle(p.name)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    out = p.with_suffix('.png')
    plt.savefig(out, dpi=150)
    print(f"Saved plot to: {out}")
    plt.show()

if __name__ == "__main__":
    main()