import heapq as hq
import networkx as nx 
import numpy as np
from config import _C as cfg

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

class PQ_2keys:
	"""
	  Implements a priority queue data structure. Each inserted item
	  has a priority associated with it and the client is usually interested
	  in quick retrieval of the lowest-priority item in the queue. This
	  data structure allows O(1) access to the lowest-priority item.
	"""
	def  __init__(self):
		self.heap = []
		self.count = 0

	def push(self, item, priority1, priority2):
		entry = (priority1, priority2, self.count, item)
		hq.heappush(self.heap, entry)
		self.count += 1

	def pop(self):
		(_, _, _, item) = hq.heappop(self.heap)
		return item

	def isEmpty(self):
		return len(self.heap) == 0

	def update(self, item, priority1, priority2):
		# If item already in priority queue with higher priority, update its priority and rebuild the heap.
		# If item already in priority queue with equal or lower priority, do nothing.
		# If item not in priority queue, do the same thing as self.push.
		for index, (p1, p2, c, i) in enumerate(self.heap):
			if i == item:
				del self.heap[index]
				self.heap.append((priority1, priority2, c, item))
				hq.heapify(self.heap)
				break
		else:
			self.push(item, priority1, priority2)

	def remove(self, item):
		for index, (p1, p2, c, i) in enumerate(self.heap):
			if i == item:
				del self.heap[index]
				break

	def topKey(self):
		item = hq.nsmallest(1, self.heap)[0]
		print(f'item = {item}')
		k1, k2, _, _ = item
		return k1, k2

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
	    if val != cfg.MAP.FREE_VAL:
	        G.remove_node(node)

	return G