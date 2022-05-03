import numpy as np 
import matplotlib.pyplot as plt 
from config import _C as cfg
import networkx as nx
import random
from utils import PriorityQueue
import math

random.seed(cfg.GENERAL.RANDOM_SEED)

class Node():
	def __init__(self, loc, parent, cost):
		self.loc = loc
		self.parent = parent # parent node
		#self.action = action # action leads from parent to current node
		self.cost = cost

class TreeList():
	def __init__(self):
		self.list = []

	def searchNode(self, searched_node):
		# check if node loc existed
		flag = False
		for node in self.list:
			if node.loc == searched_node.loc:
				flag = True
		return flag

	def insertNode(self, node):
		self.list.append(node)

	def efficientSearch(self, loc):
		start_idx = len(self.list) - 1
		idx = -1
		for i in range(start_idx, -1, -1):
			if self.list[i].loc == loc:
				idx = i
				break
		assert idx > -1
		node = self.list[idx]
		for i in range(idx-1, -1, -1):
			cur_node = self.list[i]
			if cur_node.loc == node.loc and cur_node.cost <= node.cost:
				node = cur_node
		#print(f'node.loc = {node.loc}, node.cost = {node.cost}')
		return node

	def getNode(self, loc):
		return self.efficientSearch(loc)

	def formPath(self, goal_loc):
		# the last node in the list must be a goal node
		locs = []
		'''
		for i in range(len(self.list)):
			print(f'node {i}, loc = {self.list[i].loc}, cost = {self.list[i].cost}')
		'''
		node = self.efficientSearch(goal_loc)
		
		while True:
			if node.parent is None:
				break
			locs.append(node.loc)
			parent_node = node.parent
			node = parent_node

		return locs[::-1]



def AStarSearch(start_coords, goal_coords, G):
	tree = TreeList()
	visited = []
	Q = PriorityQueue()

	start_node = Node(start_coords, None, 0.)
	goal_node = Node(goal_coords, None, 0.)
	tree.insertNode(start_node)
	Q.push(start_node.loc, 0)

	while True:
		if Q.isEmpty():
			print(f'failed to find the path ...')
			return [] # fail the search

		node_loc = Q.pop()
		node = tree.getNode(node_loc)
		if node.loc == goal_coords:
			path = tree.formPath(node_loc)
			return path
		else:
			for nei in list(G.neighbors(node.loc)):
				dist = math.sqrt((nei[0] - node_loc[0])**2 + (nei[1] - node_loc[1])**2)
				new_node = Node(nei, node, dist + node.cost)
				tree.insertNode(new_node)
				if nei not in visited:
					heur = math.sqrt((nei[0] - goal_coords[0])**2 + (nei[1] - goal_coords[1])**2)
					# update Q
					Q.update(nei, new_node.cost + heur)
			# add node to visited
			visited.append(node_loc)




occ_map_path = f'{cfg.PATH.OCC_MAP}/2t7WUuJeko7_0'
occupancy_map = np.load(f'{occ_map_path}/BEV_occupancy_map.npy', allow_pickle=True).item()['occupancy']

G = build_graph(occupancy_map)

start_coords = random.choice(list(G.nodes))
reachable_locs = list(nx.node_connected_component(G, start_coords))
end_coords = random.choice(reachable_locs)
#AStarSearch()

path = AStarSearch(start_coords, end_coords, G)
path = np.array(path) # N x 2

fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 5), dpi=125)
ax.imshow(occupancy_map, cmap='gray', vmin=0, vmax=1)
num_points = path.shape[0]
ax.scatter(path[:, 1], path[:, 0], c=range(num_points), cmap='viridis', s=np.linspace(4, 2, num=num_points)**2)
fig.tight_layout()
plt.show()