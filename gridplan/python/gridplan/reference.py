# implementaion of dijkstra's and A*
import heapq
from dataclasses import dataclass, field
import numpy as np



@dataclass
class Result:
	cost: int
	n_nodes_expanded: int
	path: list[tuple[int, int]] = field(default_factory=list)


def get_neighbors(grid, node: tuple[int, int]):

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


def dijkstra(grid, start, target):

	# need to remember that not all nodes are free; some are obstacles
	# -> use get_neighbors to get reachable nodes

	# store the unvisited nodes in a priority queue: heapq minheap
	distances = {(row, col): float('inf') for (row, col), value in np.ndenumerate(grid)}
	distances[start] = 0

	# use a dictionary to track the parent for each node
	parents = dict()
	parents[start] = None

	priority_queue = [(0, start)] 
	heapq.heapify(priority_queue)
	n_nodes_expanded = 0

	while priority_queue:
		# pop from priority queue -> smallest cost node
		cost, node = heapq.heappop(priority_queue)

		n_nodes_expanded += 1

		if cost > distances[node]:
			continue

		# find cost from this node to all of it's neighbors
		neighbors = get_neighbors(grid, node)

		for neighbor in neighbors:
			# check if the cost for each neighbor is less than the current cost for the node
			cost = cost + 1
			if cost < distances[neighbor]:
				distances[neighbor] = cost
				parents[neighbor] = node
				heapq.heappush(priority_queue, (cost, neighbor))


	# form the path list for the shortest path
	path = []

	parent = target
	while parent != None:
		path.append(parent)
		parent = parents[parent]

	path.reverse()
	cost = len(path) - 1

	return Result(cost, n_nodes_expanded, path)