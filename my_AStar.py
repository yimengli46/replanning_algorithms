import numpy as np 
import matplotlib.pyplot as plt 
from config import _C as cfg
import networkx as nx
import random
from utils import PriorityQueue, build_graph
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
				locs.append(node.loc)
				break
			locs.append(node.loc)
			parent_node = node.parent
			node = parent_node

		return locs[::-1]

class AStar:
	def __init__(self, s_start, s_goal):
		self.s_start = s_start
		self.s_goal = s_goal

		self.tree = TreeList()
		self.visited = set()
		self.Q = PriorityQueue()

		self.count_va = 0


	def computeShortestPath(self, G):
		start_node = Node(self.s_start, None, 0.)
		goal_node = Node(self.s_goal, None, 0.)
		self.tree.insertNode(start_node)
		self.Q.push(start_node.loc, 0)

		while True:
			if self.Q.isEmpty():
				print(f'failed to find the path ...')
				return [] # fail the search

			node_loc = self.Q.pop()
			node = self.tree.getNode(node_loc)
			if node.loc == self.s_goal:
				path = self.tree.formPath(node_loc)
				return path
			else:
				for nei in list(G.neighbors(node.loc)):
					dist = math.sqrt((nei[0] - node_loc[0])**2 + (nei[1] - node_loc[1])**2)
					new_node = Node(nei, node, dist + node.cost)
					self.tree.insertNode(new_node)
					if nei not in self.visited:
						heur = math.sqrt((nei[0] - self.s_goal[0])**2 + (nei[1] - self.s_goal[1])**2)
						# update Q
						self.Q.update(nei, new_node.cost + heur)
						self.count_va += 1
						#self.visited.add(nei)
				# add node to visited
				self.visited.add(node_loc)



'''
occ_map_path = f'{cfg.PATH.OCC_MAP}/2t7WUuJeko7_0'
occupancy_map = np.load(f'{occ_map_path}/BEV_occupancy_map.npy', allow_pickle=True).item()['occupancy']

G = build_graph(occupancy_map)

#start_coords = random.choice(list(G.nodes))
#reachable_locs = list(nx.node_connected_component(G, start_coords))
#end_coords = random.choice(reachable_locs)
start_coords = (23, 21)
end_coords = (76, 109)

astar = AStar(start_coords, end_coords)
path = astar.computeShortestPath(G)
path = np.array(path) # N x 2
print(f'len(path) = {len(path)}')

fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 5), dpi=125)
ax.imshow(occupancy_map, cmap='gray', vmin=0, vmax=1)
num_points = path.shape[0]
ax.scatter(path[:, 1], path[:, 0], c=range(num_points), cmap='viridis', s=np.linspace(4, 2, num=num_points)**2, zorder=2)
visited_points = np.array(list(astar.visited))
ax.scatter(visited_points[:, 1], visited_points[:, 0], c='c', marker='s')
ax.get_xaxis().set_visible(False)
ax.get_yaxis().set_visible(False)
fig.tight_layout()
plt.show()

# update occupancy map
obstacles = [(53, 83), (53, 84), (53, 85), (53, 86), (53, 87), (53, 88), (53, 89)]
for obs in obstacles:
	occupancy_map[obs] = 0
G_prime = build_graph(occupancy_map)

astar = AStar(start_coords, end_coords)
path = astar.computeShortestPath(G_prime)
path = np.array(path) # N x 2
print(f'len(path) = {len(path)}')

fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 5), dpi=125)
ax.imshow(occupancy_map, cmap='gray', vmin=0, vmax=1)
num_points = path.shape[0]
ax.scatter(path[:, 1], path[:, 0], c=range(num_points), cmap='viridis', s=np.linspace(4, 2, num=num_points)**2, zorder=2)
visited_points = np.array(list(astar.visited))
ax.scatter(visited_points[:, 1], visited_points[:, 0], c='c', marker='s')
ax.get_xaxis().set_visible(False)
ax.get_yaxis().set_visible(False)
fig.tight_layout()
plt.show()

print(f've = {len(astar.visited)}')
print(f'va = {astar.count_va}')

'''