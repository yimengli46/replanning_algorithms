import numpy as np 
import matplotlib.pyplot as plt 
from config import _C as cfg
import networkx as nx
import random
import heapq as hq
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

class PriorityQueue:
	"""
	  Implements a priority queue data structure. Each inserted item
	  has a priority associated with it and the client is usually interested
	  in quick retrieval of the lowest-priority item in the queue. This
	  data structure allows O(1) access to the lowest-priority item.
	"""
	def  __init__(self):
		self.heap = []
		self.count = 0

	def push(self, item, priority):
		entry = (priority, self.count, item)
		hq.heappush(self.heap, entry)
		self.count += 1

	def pop(self):
		(_, _, item) = hq.heappop(self.heap)
		return item

	def isEmpty(self):
		return len(self.heap) == 0

	def update(self, item, priority):
		# If item already in priority queue with higher priority, update its priority and rebuild the heap.
		# If item already in priority queue with equal or lower priority, do nothing.
		# If item not in priority queue, do the same thing as self.push.
		for index, (p, c, i) in enumerate(self.heap):
			if i == item:
				if p <= priority:
					break
				del self.heap[index]
				self.heap.append((priority, c, item))
				hq.heapify(self.heap)
				break
		else:
			self.push(item, priority)

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

def build_graph(occupancy_map):
	H, W = occupancy_map.shape
	G = nx.grid_2d_graph(*occupancy_map.shape)

	G.add_edges_from([
	    ((x, y), (x+1, y+1))
	    for x in range(1, H-1)
	    for y in range(1, W-1)
	] + [
	    ((x, y), (x-1, y-1))
	    for x in range(1, H-1)
	    for y in range(1, W-1)
	] + [
	    ((x, y), (x-1, y+1))
	    for x in range(1, H-1)
	    for y in range(1, W-1)
	] + [
	    ((x, y), (x+1, y-1))
	    for x in range(1, H-1)
	    for y in range(1, W-1)
	])

	# remove those nodes where the corresponding value is != 0
	for val, node in zip(occupancy_map.ravel(), sorted(G.nodes())):
	    if val!=cfg.MAP.COLLISION_VAL:
	        G.remove_node(node)

	return G


occ_map_path = f'{cfg.PATH.OCC_MAP}/2t7WUuJeko7_0'
occupancy_map = np.load(f'{occ_map_path}/BEV_occupancy_map.npy', allow_pickle=True).item()['occupancy']

G = build_graph(occupancy_map)

start_coords = random.choice(list(G.nodes))
reachable_locs = list(nx.node_connected_component(G, start_coords))
end_coords = random.choice(reachable_locs)
#AStarSearch()

path = AStarSearch(start_coords, end_coords, G)