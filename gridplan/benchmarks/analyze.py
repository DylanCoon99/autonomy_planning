"""
Analyze benchmark results and generate summary plots.

Usage: python benchmarks/analyze.py [--input benchmarks/results.csv] [--outdir benchmarks/plots/]
"""

import argparse
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def load_data(csv_path):
    df = pd.read_csv(csv_path)
    # Ensure consistent types
    df['grid_size'] = df['grid_size'].astype(int)
    df['density'] = df['density'].astype(float)
    df['weight'] = df['weight'].astype(float)
    df['path_cost'] = df['path_cost'].astype(float)
    df['nodes_expanded'] = df['nodes_expanded'].astype(int)
    df['python_e2e_time_ms'] = df['python_e2e_time_ms'].astype(float)
    return df


def plot_expansion_reduction(df, outdir):
    """A* expansion reduction vs Dijkstra by obstacle density."""
    cpp_2d = df[(df['dim'] == '2D') & (df['implementation'] == 'cpp')]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for ax, conn in zip(axes, ['FOUR', 'EIGHT']):
        data = cpp_2d[cpp_2d['connectivity'] == conn]
        densities = sorted(data['density'].unique())

        dijkstra_medians = []
        astar_medians = []

        for d in densities:
            dij = data[(data['algorithm'] == 'Dijkstra') & (data['density'] == d)]
            ast = data[(data['algorithm'] == 'AStar') & (data['density'] == d) &
                       (data['weight'] == 1.0) & (data['tie_break'] == False)]
            dijkstra_medians.append(dij['nodes_expanded'].median())
            astar_medians.append(ast['nodes_expanded'].median())

        x = np.arange(len(densities))
        width = 0.35
        ax.bar(x - width/2, dijkstra_medians, width, label='Dijkstra', color='steelblue')
        ax.bar(x + width/2, astar_medians, width, label='A*', color='coral')
        ax.set_xlabel('Obstacle Density')
        ax.set_ylabel('Median Nodes Expanded')
        ax.set_title(f'{conn}-connected')
        ax.set_xticks(x)
        ax.set_xticklabels([str(d) for d in densities])
        ax.legend()
        ax.set_yscale('log')

    plt.suptitle('A* Expansion Reduction vs Dijkstra by Obstacle Density', fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, 'expansion_reduction.png'), dpi=150, bbox_inches='tight')
    plt.close()


def plot_weighted_suboptimality(df, outdir):
    """Weighted A* empirical suboptimality vs theoretical bound."""
    cpp_2d = df[(df['dim'] == '2D') & (df['implementation'] == 'cpp') & (df['connectivity'] == 'FOUR')]

    # Get optimal costs from Dijkstra
    dijkstra = cpp_2d[cpp_2d['algorithm'] == 'Dijkstra'][['grid_size', 'density', 'seed', 'path_cost']]
    dijkstra = dijkstra.rename(columns={'path_cost': 'optimal_cost'})

    weighted = cpp_2d[cpp_2d['algorithm'] == 'WeightedAStar']
    merged = weighted.merge(dijkstra, on=['grid_size', 'density', 'seed'])
    merged['suboptimality'] = merged['path_cost'] / merged['optimal_cost']

    fig, ax = plt.subplots(figsize=(10, 6))

    weights = sorted(merged['weight'].unique())
    densities = sorted(merged['density'].unique())

    for density in densities:
        sub = merged[merged['density'] == density]
        medians = [sub[sub['weight'] == w]['suboptimality'].median() for w in weights]
        ax.plot(weights, medians, 'o-', label=f'density={density}')

    # Theoretical bound line
    ax.plot(weights, weights, 'k--', linewidth=2, label='Theoretical bound (w)')

    ax.set_xlabel('Weight (w)')
    ax.set_ylabel('Suboptimality Ratio (cost / optimal)')
    ax.set_title('Weighted A* Empirical Suboptimality vs Theoretical Bound')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.savefig(os.path.join(outdir, 'weighted_suboptimality.png'), dpi=150, bbox_inches='tight')
    plt.close()


def plot_tiebreaking(df, outdir):
    """Tie-breaking effect on open vs cluttered grids."""
    cpp_2d = df[(df['dim'] == '2D') & (df['implementation'] == 'cpp') &
                (df['algorithm'] == 'AStar') & (df['weight'] == 1.0)]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for ax, conn in zip(axes, ['FOUR', 'EIGHT']):
        data = cpp_2d[cpp_2d['connectivity'] == conn]
        densities = sorted(data['density'].unique())

        no_tb = []
        with_tb = []

        for d in densities:
            nt = data[(data['density'] == d) & (data['tie_break'] == False)]
            wt = data[(data['density'] == d) & (data['tie_break'] == True)]
            no_tb.append(nt['nodes_expanded'].median())
            with_tb.append(wt['nodes_expanded'].median())

        x = np.arange(len(densities))
        width = 0.35
        ax.bar(x - width/2, no_tb, width, label='No tie-breaking', color='steelblue')
        ax.bar(x + width/2, with_tb, width, label='With tie-breaking', color='coral')
        ax.set_xlabel('Obstacle Density')
        ax.set_ylabel('Median Nodes Expanded')
        ax.set_title(f'{conn}-connected')
        ax.set_xticks(x)
        ax.set_xticklabels([str(d) for d in densities])
        ax.legend()

    plt.suptitle('Effect of Tie-Breaking on A* Node Expansions', fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, 'tiebreaking.png'), dpi=150, bbox_inches='tight')
    plt.close()


def plot_2d_vs_3d_scaling(df, outdir):
    """Runtime scaling from 2D to 3D."""
    cpp = df[(df['implementation'] == 'cpp') & (df['algorithm'] == 'Dijkstra')]

    fig, ax = plt.subplots(figsize=(10, 6))

    # 2D
    d2 = cpp[cpp['dim'] == '2D']
    for conn in d2['connectivity'].unique():
        sub = d2[d2['connectivity'] == conn]
        sizes = sorted(sub['grid_size'].unique())
        times = [sub[sub['grid_size'] == s]['python_e2e_time_ms'].median() for s in sizes]
        labels = [f'{s}²' for s in sizes]
        ax.plot(range(len(sizes)), times, 'o-', label=f'2D {conn}')
        for i, (lbl, t) in enumerate(zip(labels, times)):
            ax.annotate(lbl, (i, t), textcoords="offset points", xytext=(0, 8), ha='center', fontsize=8)

    # 3D
    d3 = cpp[cpp['dim'] == '3D']
    for conn in d3['connectivity'].unique():
        sub = d3[d3['connectivity'] == conn]
        sizes = sorted(sub['grid_size'].unique())
        times = [sub[sub['grid_size'] == s]['python_e2e_time_ms'].median() for s in sizes]
        labels = [f'{s}³' for s in sizes]
        offset = len(d2['grid_size'].unique())
        ax.plot(range(offset, offset + len(sizes)), times, 's--', label=f'3D {conn}')
        for i, (lbl, t) in enumerate(zip(labels, times)):
            ax.annotate(lbl, (offset + i, t), textcoords="offset points", xytext=(0, 8), ha='center', fontsize=8)

    ax.set_ylabel('Median Time (ms)')
    ax.set_title('Dijkstra Runtime: 2D vs 3D Scaling')
    ax.legend()
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)
    ax.set_xticks([])
    plt.savefig(os.path.join(outdir, '2d_vs_3d_scaling.png'), dpi=150, bbox_inches='tight')
    plt.close()


def plot_cpp_vs_python_speedup(df, outdir):
    """C++ speedup over Python reference by grid size."""
    four_conn = df[(df['connectivity'] == 'FOUR') & (df['dim'] == '2D')]

    fig, ax = plt.subplots(figsize=(10, 6))

    for algo in ['Dijkstra', 'AStar']:
        cpp = four_conn[(four_conn['implementation'] == 'cpp') & (four_conn['algorithm'] == algo) &
                        (four_conn['weight'] == 1.0) & (four_conn['tie_break'] == False)]
        py = four_conn[(four_conn['implementation'] == 'python') & (four_conn['algorithm'] == algo)]

        # Only sizes where both exist
        common_sizes = sorted(set(cpp['grid_size'].unique()) & set(py['grid_size'].unique()))

        if not common_sizes:
            continue

        speedups = []
        for s in common_sizes:
            cpp_time = cpp[cpp['grid_size'] == s]['python_e2e_time_ms'].median()
            py_time = py[py['grid_size'] == s]['python_e2e_time_ms'].median()
            if cpp_time > 0:
                speedups.append(py_time / cpp_time)
            else:
                speedups.append(0)

        ax.plot(range(len(common_sizes)), speedups, 'o-', label=algo, markersize=8)
        ax.set_xticks(range(len(common_sizes)))
        ax.set_xticklabels([f'{s}²' for s in common_sizes])

    ax.set_xlabel('Grid Size')
    ax.set_ylabel('Speedup (Python time / C++ time)')
    ax.set_title('C++ Speedup over Python Reference')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.savefig(os.path.join(outdir, 'cpp_vs_python_speedup.png'), dpi=150, bbox_inches='tight')
    plt.close()


def plot_binding_overhead(df, outdir):
    """Binding/conversion overhead fraction by problem size."""
    cpp_2d = df[(df['dim'] == '2D') & (df['implementation'] == 'cpp') &
                (df['algorithm'] == 'Dijkstra')]

    fig, ax = plt.subplots(figsize=(10, 6))

    for conn in ['FOUR', 'EIGHT']:
        data = cpp_2d[cpp_2d['connectivity'] == conn]
        sizes = sorted(data['grid_size'].unique())

        overhead_fracs = []
        for s in sizes:
            sub = data[data['grid_size'] == s]
            e2e = sub['python_e2e_time_ms'].median()
            cpp_t = sub['cpp_time_ms'].median()
            if e2e > 0:
                overhead_fracs.append((e2e - cpp_t) / e2e * 100)
            else:
                overhead_fracs.append(0)

        ax.plot(range(len(sizes)), overhead_fracs, 'o-', label=f'{conn}-connected', markersize=8)
        ax.set_xticks(range(len(sizes)))
        ax.set_xticklabels([f'{s}²' for s in sizes])

    ax.set_xlabel('Grid Size')
    ax.set_ylabel('Binding Overhead (%)')
    ax.set_title('Binding/Conversion Overhead as Fraction of End-to-End Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)
    plt.savefig(os.path.join(outdir, 'binding_overhead.png'), dpi=150, bbox_inches='tight')
    plt.close()


def plot_runtime_by_algorithm(df, outdir):
    """Runtime comparison across all algorithms by grid size."""
    cpp_2d = df[(df['dim'] == '2D') & (df['implementation'] == 'cpp') &
                (df['connectivity'] == 'FOUR') & (df['tie_break'] == False)]

    fig, ax = plt.subplots(figsize=(12, 7))

    algos = [
        ('BFS', 'BFS', 1.0),
        ('Dijkstra', 'Dijkstra', 1.0),
        ('AStar', 'A* (w=1.0)', 1.0),
        ('WeightedAStar', 'Weighted A* (w=2.0)', 2.0),
        ('WeightedAStar', 'Weighted A* (w=5.0)', 5.0),
    ]

    sizes = sorted(cpp_2d['grid_size'].unique())

    for algo_name, label, weight in algos:
        data = cpp_2d[(cpp_2d['algorithm'] == algo_name) & (cpp_2d['weight'] == weight)]
        times = [data[data['grid_size'] == s]['python_e2e_time_ms'].median() for s in sizes]
        ax.plot(range(len(sizes)), times, 'o-', label=label, markersize=8)

    ax.set_xticks(range(len(sizes)))
    ax.set_xticklabels([f'{s}²' for s in sizes])
    ax.set_xlabel('Grid Size')
    ax.set_ylabel('Median Time (ms)')
    ax.set_title('Runtime by Algorithm (2D, FOUR-connected, density mixed)')
    ax.legend()
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3)
    plt.savefig(os.path.join(outdir, 'runtime_by_algorithm.png'), dpi=150, bbox_inches='tight')
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="Analyze benchmark results")
    parser.add_argument("--input", default="benchmarks/results.csv", help="Input CSV path")
    parser.add_argument("--outdir", default="benchmarks/plots", help="Output directory for plots")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    print(f"Loading {args.input}...")
    df = load_data(args.input)
    print(f"Loaded {len(df)} rows")

    print("Generating plots...")

    plot_expansion_reduction(df, args.outdir)
    print("  - expansion_reduction.png")

    plot_weighted_suboptimality(df, args.outdir)
    print("  - weighted_suboptimality.png")

    plot_tiebreaking(df, args.outdir)
    print("  - tiebreaking.png")

    plot_2d_vs_3d_scaling(df, args.outdir)
    print("  - 2d_vs_3d_scaling.png")

    plot_cpp_vs_python_speedup(df, args.outdir)
    print("  - cpp_vs_python_speedup.png")

    plot_binding_overhead(df, args.outdir)
    print("  - binding_overhead.png")

    plot_runtime_by_algorithm(df, args.outdir)
    print("  - runtime_by_algorithm.png")

    print(f"\nAll plots saved to {args.outdir}/")


if __name__ == "__main__":
    main()
