import numpy as np


class Environment:

    def __init__(self, dimensions: tuple[int, int], obstacle_density: float, seed: int):
        self.dimensions = dimensions
        self.obstacle_density = obstacle_density
        self.rng = np.random.default_rng(seed=seed)
        self.start = None
        self.target = None
        self.grid = self._create_grid()  # np.ndarray of uint8
    
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
    
    
    def _create_grid(self):
        # TODO: method to create a grid
        # implement grid as a numpy array with 0 representing free cell and 1 representing an obstacle
        rows, cols = self.dimensions[0], self.dimensions[1]
        n_cells = rows * cols
        n_obstacles = int(n_cells * self.obstacle_density)
        grid = np.zeros(n_cells, dtype=np.uint8)
        grid[:n_obstacles] = 1
        rng.shuffle(grid)
        grid = grid.reshape((rows, cols))
        
        
        # set start and target randomly
        
        return grid
        
    def print_grid(self):
    
        print("Printing grid...")
        print(self.grid)
        

    def _validate_grid(self, grid, start, target):
        # TODO: validates the target is reachable from the start
        pass



def create_environment(dimensions: tuple[int, int], obstacle_density: int, seed: int):
    # creates an environment for the provided parameters
    return Environment(dimensions, obstacle_density, seed)
