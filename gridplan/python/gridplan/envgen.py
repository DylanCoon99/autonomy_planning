import numpy as np
from scipy import ndimage
from collections import deque


class Environment:

	def __init__(self, dimensions: tuple[int, int], obstacle_density: float, seed: int, option="uniform", obstacle_inflation=False):
		self.dimensions = dimensions
		self.obstacle_density = obstacle_density
		self.rng = np.random.default_rng(seed=seed)
		self.start = None
		self.target = None
		self.obstacle_inflation = obstacle_inflation
		match option:
			case "uniform":
				self.grid = self._create_grid_uniform()
			case "box":
				self.grid = self._create_grid_box()
			case "maze":
				self.grid = self._create_grid_maze()
			case _:
				raise ValueError(f"Invalid setting: {option} Choose 'uniform', 'box', or 'maze' for the environment setting.")

	@property
	def dimensions(self) -> tuple[int, int]:
		return self._dimensions

	@dimensions.setter
	def dimensions(self, value: tuple[int, int]):
		# validate the width and height before setting
		is_valid_format = (
			isinstance(value, tuple)
			and len(value) == 2
			and isinstance(value[0], int)
			and isinstance(value[1], int)
		)
		if not is_valid_format:
			raise TypeError("Dimensions must be a tuple of shape [int, int].")

		is_valid_range = (
			value[0] >= 2
			and value[1] >= 2
		)
		if not is_valid_range:
			raise ValueError("Dimensions (width, height) must be greater than 2.")

		self._dimensions = value

	@property
	def obstacle_density(self) -> float:
		return self._obstacle_density

	@obstacle_density.setter
	def obstacle_density(self, value: float):
		# validate obstacle_density is float
		if not isinstance(value, float):
			raise TypeError("Obstacle density must be of type float.")
		if not (value >= 0 and value < 1):
			raise ValueError("Obstacle density must be in range (0.0 to 1.0)")

		self._obstacle_density = value

	def _create_grid_uniform(self):
		# implement grid as a numpy array with 0 representing free cell and 1 representing an obstacle

		rows, cols = self.dimensions[0], self.dimensions[1]
		n_cells = rows * cols
		n_obstacles = int(n_cells * self.obstacle_density)

		valid_grid = False

		while (not valid_grid):
			grid = np.zeros(n_cells, dtype=np.uint8)
			grid[:n_obstacles] = 1
			self.rng.shuffle(grid)
			grid = grid.reshape((rows, cols))

			if self.obstacle_inflation:
				# TODO: obstacle inflation
				grid = (ndimage.binary_dilation(grid)).astype(int)

			# TODO: set the start and target
			self.start = np.unravel_index(np.argmin(grid != 0), grid.shape)
			idx = (grid == 0).size - 1 - np.argmax((grid == 0).flat[::-1])
			self.target = np.unravel_index(idx, grid.shape)

			valid_grid = self._validate_grid(grid, self.start, self.target)

		return grid


	def _create_grid_box(self):

		return


	def _create_grid_maze(self):

		return

	def get_neighbors(self, grid, node: tuple[int, int]):

		# each node has up to 4 reachable neighbors
		# up, down, left, right
		neighbors = []
		max_row, max_col = grid.shape

		# check each to make sure they are not out of bounds and they are equal to 0
		if node[0] > 0:
			# up is reachable
			idx, idy = node[0] - 1, node[1]
			if grid[idx][idy] == 0:
				neighbors.append((idx, idy))
		if node[0] != max_row - 1:
			# down is reachable
			idx, idy = node[0] + 1, node[1]
			if grid[idx][idy] == 0:
				neighbors.append((idx, idy))
		if node[1] > 0:
			# left is reachable
			idx, idy = node[0], node[1] - 1
			if grid[idx][idy] == 0:
				neighbors.append((idx, idy))
		if node[1] != max_col - 1:
			# left is reachable
			idx, idy = node[0], node[1] + 1
			if grid[idx][idy] == 0:
				neighbors.append((idx, idy))

		return neighbors


	def _validate_grid(self, grid: tuple[int, int], start: tuple[int, int], target) -> bool:
		# TODO: validates the target is reachable from the start; uses DFS
		visited = set()
		stack = deque([start])

		visited.add(start)

		while len(stack) > 0:
			# pop top node from the stack
			node = stack.pop()

			# get reachable neighbors for a node
			neighbors = self.get_neighbors(grid, node)

			# add the node's neighbors to the stack if not already in visited
			for neighbor in neighbors:
				if neighbor not in visited:
					visited.add(neighbor)
					stack.append(neighbor)

		return target in visited


	def print_grid(self):

		print(f"Start: {self.start}")
		print(f"Target: {self.target}")

		print("Printing grid...")
		print(self.grid)


def create_environment(dimensions: tuple[int, int], obstacle_density: int, seed: int, option="uniform", obstacle_inflation=False):
	# creates an environment for the provided parameters
	return Environment(dimensions, obstacle_density, seed, option=option, obstacle_inflation=obstacle_inflation)

