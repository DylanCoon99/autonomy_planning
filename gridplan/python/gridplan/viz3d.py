import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import gridplan


def _padded_index_3d(row, col, depth, cols, rows):
    padded_cols = cols + 2
    z_stride = (rows + 2) * padded_cols
    return (depth + 1) * z_stride + (row + 1) * padded_cols + (col + 1)


def _generate_3d_grid(shape, density, seed):
    """Generate a random 3D occupancy grid with guaranteed reachable path."""
    rng = np.random.default_rng(seed=seed)
    rows, cols, depth = shape
    n_cells = rows * cols * depth
    n_obstacles = int(n_cells * density)

    grid = np.zeros(n_cells, dtype=np.uint8)
    grid[:n_obstacles] = 1
    rng.shuffle(grid)
    grid = grid.reshape((depth, rows, cols))

    # Clear start and goal corners
    grid[0, 0, 0] = 0
    grid[-1, -1, -1] = 0

    return grid


def _draw_voxel_cube(ax, pos, color='gray', alpha=0.3):
    """Draw a single cube at position (x, y, z)."""
    x, y, z = pos
    vertices = [
        [x, y, z], [x+1, y, z], [x+1, y+1, z], [x, y+1, z],
        [x, y, z+1], [x+1, y, z+1], [x+1, y+1, z+1], [x, y+1, z+1]
    ]
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[0], vertices[3], vertices[7], vertices[4]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
    ]
    collection = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor='darkgray', linewidth=0.3)
    ax.add_collection3d(collection)


def _is_surface_voxel(grid, z, r, c):
    """Check if a voxel has at least one free or out-of-bounds neighbor (surface voxel)."""
    depth, rows, cols = grid.shape
    for dz, dr, dc in [(-1,0,0),(1,0,0),(0,-1,0),(0,1,0),(0,0,-1),(0,0,1)]:
        nz, nr, nc = z + dz, r + dr, c + dc
        if nz < 0 or nz >= depth or nr < 0 or nr >= rows or nc < 0 or nc >= cols:
            return True
        if grid[nz, nr, nc] == 0:
            return True
    return False


def render_3d_grid(grid_3d, path_coords=None, title="3D Voxel Grid", save_path=None,
                   surface_only=True, obstacle_color='steelblue', obstacle_alpha=0.2,
                   path_color='red', path_linewidth=3, figsize=(12, 10)):
    """Render a 3D voxel grid with optional path overlay.

    Args:
        grid_3d: 3D numpy array (depth, rows, cols), 1=obstacle, 0=free
        path_coords: list of (row, col, depth) tuples for the path, or None
        title: plot title
        save_path: if set, save figure to this path
        surface_only: only render surface voxels (much faster for large grids)
        obstacle_color: color for obstacle voxels
        obstacle_alpha: transparency for obstacle voxels
        path_color: color for the path line
        path_linewidth: width of path line
        figsize: figure size
    """
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')

    depth, rows, cols = grid_3d.shape

    # Draw obstacle voxels
    for z in range(depth):
        for r in range(rows):
            for c in range(cols):
                if grid_3d[z, r, c] == 1:
                    if surface_only and not _is_surface_voxel(grid_3d, z, r, c):
                        continue
                    _draw_voxel_cube(ax, (c, r, z), color=obstacle_color, alpha=obstacle_alpha)

    # Draw path
    if path_coords and len(path_coords) > 0:
        pr = [coord[0] + 0.5 for coord in path_coords]
        pc = [coord[1] + 0.5 for coord in path_coords]
        pz = [coord[2] + 0.5 for coord in path_coords]
        ax.plot(pc, pr, pz, color=path_color, linewidth=path_linewidth, zorder=10)

        # Mark start and goal
        ax.scatter([pc[0]], [pr[0]], [pz[0]], color='green', s=100, zorder=11, label='Start')
        ax.scatter([pc[-1]], [pr[-1]], [pz[-1]], color='red', s=100, marker='*', zorder=11, label='Goal')

    ax.set_xlabel('X (col)')
    ax.set_ylabel('Y (row)')
    ax.set_zlabel('Z (depth)')
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_zlim(0, depth)
    ax.set_title(title)
    ax.legend()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved to {save_path}")
    plt.show()


def plan_and_render_3d(shape=(16, 16, 16), density=0.15, seed=42, connectivity=gridplan.Connectivity.SIX,
                       heuristic=gridplan.Heuristic.MANHATTAN, save_path=None):
    """Generate a 3D grid, plan a path with A*, and render the result.

    Args:
        shape: (rows, cols, depth)
        density: obstacle density
        seed: random seed
        connectivity: grid connectivity mode
        heuristic: heuristic for A*
        save_path: if set, save the figure
    """
    rows, cols, depth = shape
    grid_3d = _generate_3d_grid(shape, density, seed)

    # Create C++ grid and plan
    flat = grid_3d.flatten().tolist()
    cpp_grid = gridplan.Grid(rows, cols, depth, flat, connectivity)

    start_idx = _padded_index_3d(0, 0, 0, cols, rows)
    goal_idx = _padded_index_3d(rows - 1, cols - 1, depth - 1, cols, rows)

    config = gridplan.PlannerConfig(heuristic=heuristic)
    astar = gridplan.AStar()
    result = astar.plan(cpp_grid, start_idx, goal_idx, config)

    if result.cost < 0:
        print("No path found. Try a lower density or different seed.")
        return None

    # Convert path to coordinates
    path_coords = []
    for idx in result.path:
        r, c, z = cpp_grid.index_to_coords(idx)
        path_coords.append((r, c, z))

    title = (f"3D A* Path ({rows}x{cols}x{depth}, density={density})\n"
             f"Cost: {result.cost:.2f} | Expanded: {result.nodes_expanded}")

    render_3d_grid(grid_3d, path_coords=path_coords, title=title, save_path=save_path)

    return result
