import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ListedColormap
import os

import gridplan
from gridplan.envgen import Environment


def _padded_index(row, col, cols):
    return (row + 1) * (cols + 2) + (col + 1)


def _run_planner(env, planner_class, config):
    """Run a C++ planner on an Environment and return the PlannerResult."""
    rows, cols = env.dimensions
    flat = env.grid.flatten().tolist()
    cpp_grid = gridplan.Grid(rows, cols, flat, gridplan.Connectivity.FOUR)

    config.record_expansions = True

    start_idx = _padded_index(env.start[0], env.start[1], cols)
    goal_idx = _padded_index(env.target[0], env.target[1], cols)

    planner = planner_class()
    result = planner.plan(cpp_grid, start_idx, goal_idx, config)
    return result, cpp_grid


def _expansion_to_grid(result, rows, cols, cpp_grid):
    """Convert expansion_order (padded indices) to a 2D array of step numbers."""
    expansion_grid = np.full((rows, cols), np.nan)
    if result.expansion_order is not None:
        for step, idx in enumerate(result.expansion_order):
            r, c, _ = cpp_grid.index_to_coords(idx)
            if 0 <= r < rows and 0 <= c < cols:
                expansion_grid[r, c] = step
    return expansion_grid


def _path_to_coords(result, cpp_grid):
    """Convert path (padded indices) to lists of row and col coordinates."""
    path_rows = []
    path_cols = []
    for idx in result.path:
        r, c, _ = cpp_grid.index_to_coords(idx)
        path_rows.append(r)
        path_cols.append(c)
    return path_rows, path_cols


def plot_expansion_heatmap(env, planner_class, config, title=None, ax=None, save_path=None):
    """Plot expansion-order heatmap with path overlay."""
    result, cpp_grid = _run_planner(env, planner_class, config)
    rows, cols = env.dimensions

    expansion_grid = _expansion_to_grid(result, rows, cols, cpp_grid)
    path_rows, path_cols = _path_to_coords(result, cpp_grid)

    if ax is None:
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        standalone = True
    else:
        standalone = False

    # Draw obstacles in black
    obstacle_display = np.where(env.grid == 1, 0.0, np.nan)
    ax.imshow(obstacle_display, cmap=ListedColormap(['black']), interpolation='nearest',
              extent=(-0.5, cols - 0.5, rows - 0.5, -0.5))

    # Draw expansion order heatmap
    im = ax.imshow(expansion_grid, cmap='viridis', interpolation='nearest',
                   extent=(-0.5, cols - 0.5, rows - 0.5, -0.5), alpha=0.7)

    # Draw path
    ax.plot(path_cols, path_rows, color='red', linewidth=2, zorder=3)

    # Mark start and goal
    ax.plot(env.start[1], env.start[0], 'go', markersize=10, zorder=4, label='Start')
    ax.plot(env.target[1], env.target[0], 'r*', markersize=14, zorder=4, label='Goal')

    if title is None:
        title = planner_class.__name__
    ax.set_title(f"{title}\nCost: {result.cost:.2f} | Expanded: {result.nodes_expanded}", fontsize=11)
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)
    ax.set_aspect('equal')

    if standalone:
        plt.colorbar(im, ax=ax, label='Expansion order')
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved to {save_path}")
        plt.show()

    return result


def plot_comparison(env, save_path=None):
    """Side-by-side comparison of BFS, Dijkstra, A*, and weighted A* on one grid."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 14))

    planners = [
        (gridplan.BFS, gridplan.PlannerConfig(), "BFS"),
        (gridplan.Dijkstra, gridplan.PlannerConfig(), "Dijkstra"),
        (gridplan.AStar, gridplan.PlannerConfig(heuristic=gridplan.Heuristic.MANHATTAN), "A* (Manhattan)"),
        (gridplan.AStar, gridplan.PlannerConfig(weight=2.0, heuristic=gridplan.Heuristic.MANHATTAN), "Weighted A* (w=2.0)"),
    ]

    for ax, (planner_class, config, title) in zip(axes.flat, planners):
        result, cpp_grid = _run_planner(env, planner_class, config)
        rows, cols = env.dimensions

        expansion_grid = _expansion_to_grid(result, rows, cols, cpp_grid)
        path_rows, path_cols = _path_to_coords(result, cpp_grid)

        obstacle_display = np.where(env.grid == 1, 0.0, np.nan)
        ax.imshow(obstacle_display, cmap=ListedColormap(['black']), interpolation='nearest',
                  extent=(-0.5, cols - 0.5, rows - 0.5, -0.5))

        im = ax.imshow(expansion_grid, cmap='viridis', interpolation='nearest',
                       extent=(-0.5, cols - 0.5, rows - 0.5, -0.5), alpha=0.7)

        ax.plot(path_cols, path_rows, color='red', linewidth=2, zorder=3)
        ax.plot(env.start[1], env.start[0], 'go', markersize=8, zorder=4)
        ax.plot(env.target[1], env.target[0], 'r*', markersize=12, zorder=4)

        ax.set_title(f"{title}\nCost: {result.cost:.2f} | Expanded: {result.nodes_expanded}", fontsize=11)
        ax.set_xlim(-0.5, cols - 0.5)
        ax.set_ylim(rows - 0.5, -0.5)
        ax.set_aspect('equal')

    plt.suptitle(f"Planner Comparison ({env.dimensions[0]}x{env.dimensions[1]}, density={env.obstacle_density})",
                 fontsize=14, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved to {save_path}")
    plt.show()


def animate_expansion(env, planner_class, config, save_path="expansion.gif", interval=20, step_size=1):
    """Animate the expansion order frame by frame, then draw the path."""
    result, cpp_grid = _run_planner(env, planner_class, config)
    rows, cols = env.dimensions

    if result.expansion_order is None or len(result.expansion_order) == 0:
        print("No expansion order recorded.")
        return

    # Precompute all expansion coordinates
    expansion_coords = []
    for idx in result.expansion_order:
        r, c, _ = cpp_grid.index_to_coords(idx)
        if 0 <= r < rows and 0 <= c < cols:
            expansion_coords.append((r, c))

    path_rows, path_cols = _path_to_coords(result, cpp_grid)

    fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    # Base grid: obstacles in black, free in white
    base = np.ones((rows, cols, 3))  # white
    for r in range(rows):
        for c in range(cols):
            if env.grid[r, c] == 1:
                base[r, c] = [0, 0, 0]  # black

    img = ax.imshow(base, interpolation='nearest')
    ax.plot(env.start[1], env.start[0], 'go', markersize=10, zorder=4)
    ax.plot(env.target[1], env.target[0], 'r*', markersize=14, zorder=4)
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)
    ax.set_aspect('equal')

    title_text = ax.set_title(f"{planner_class.__name__} - Step 0/{len(expansion_coords)}")

    # Color map for expansion
    cmap = plt.cm.viridis
    n_expansion_frames = (len(expansion_coords) + step_size - 1) // step_size
    n_path_frames = 10
    total_frames = n_expansion_frames + n_path_frames

    path_line, = ax.plot([], [], color='red', linewidth=2, zorder=3)

    display = base.copy()

    def update(frame):
        nonlocal display

        if frame < n_expansion_frames:
            # Expansion phase: color cells
            start_step = frame * step_size
            end_step = min(start_step + step_size, len(expansion_coords))
            for i in range(start_step, end_step):
                r, c = expansion_coords[i]
                norm_val = i / max(len(expansion_coords) - 1, 1)
                color = cmap(norm_val)[:3]
                display[r, c] = color
            img.set_data(display)
            title_text.set_text(f"{planner_class.__name__} - Step {end_step}/{len(expansion_coords)}")
        else:
            # Path phase: draw path progressively
            path_frame = frame - n_expansion_frames
            n_points = int((path_frame + 1) / n_path_frames * len(path_rows))
            n_points = max(1, min(n_points, len(path_rows)))
            path_line.set_data(path_cols[:n_points], path_rows[:n_points])
            title_text.set_text(
                f"{planner_class.__name__} - Cost: {result.cost:.2f} | Expanded: {result.nodes_expanded}")

        return [img, path_line, title_text]

    anim = animation.FuncAnimation(fig, update, frames=total_frames, interval=interval, blit=True)

    if save_path.endswith('.gif'):
        anim.save(save_path, writer='pillow')
    elif save_path.endswith('.mp4'):
        anim.save(save_path, writer='ffmpeg')
    else:
        anim.save(save_path, writer='pillow')

    plt.close(fig)
    print(f"Saved animation to {save_path}")


def generate_all_animations(env, output_dir="animations/", interval=20, step_size=10):
    """Generate one expansion animation GIF per algorithm."""
    os.makedirs(output_dir, exist_ok=True)

    planners = [
        (gridplan.BFS, gridplan.PlannerConfig(), "bfs"),
        (gridplan.Dijkstra, gridplan.PlannerConfig(), "dijkstra"),
        (gridplan.AStar, gridplan.PlannerConfig(heuristic=gridplan.Heuristic.MANHATTAN), "astar"),
        (gridplan.AStar, gridplan.PlannerConfig(weight=2.0, heuristic=gridplan.Heuristic.MANHATTAN), "weighted_astar_w2"),
    ]

    for planner_class, config, name in planners:
        save_path = os.path.join(output_dir, f"{name}.gif")
        print(f"Generating {name}...")
        animate_expansion(env, planner_class, config, save_path=save_path,
                          interval=interval, step_size=step_size)

    print(f"All animations saved to {output_dir}")
