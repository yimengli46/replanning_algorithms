import numpy as np 
import matplotlib.pyplot as plt 
from config import _C as cfg
import networkx as nx
import random
from utils import PQ_2keys, build_graph
import math

random.seed(cfg.GENERAL.RANDOM_SEED)

class DStarLite:
	def __init__(self, s_start, s_goal):
		self.g = {}
		self.rhs = {}
		self.U = PQ_2keys()

		self.s_start = s_start
		self.s_goal = s_goal

		self.visited = set()
		self.count = 0


	def initialize(self, G):
		assert self.U.isEmpty()

		nodes = list(G.nodes)
		for node in nodes:
			self.rhs[node] = float('inf')
			self.g[node] = float('inf')

		self.rhs[self.s_goal] = 0
		k1, k2 = self.calculateKey(self.s_goal)
		self.U.push(self.s_goal, k1, k2)


	def calculateKey(self, s):
		k1 = min(self.g[s], self.rhs[s]) + self.h(s)
		k2 = min(self.g[s], self.rhs[s])
		return k1, k2


	def h(self, s):
		goal = self.s_goal
		dist = math.sqrt((s[0] - goal[0])**2 + (s[1] - goal[1])**2)
		return dist

	def updateVertex(self, s, G):
		if s != self.s_goal:
			self.rhs[s] = float('inf')
			neis = list(G.neighbors(s))
			for s_n in neis:
				self.rhs[s] = min(self.rhs[s], self.g[s_n] + self.cost(s, s_n))

		self.U.remove(s)

		#print(f's = {s}, g[s] = {self.g[s]}, rhs = {self.rhs[s]}')
		if self.g[s] != self.rhs[s]:
			k1, k2 = self.calculateKey(s)
			self.U.update(s, k1, k2)


	def cost(self, s1, s2):
		dist = math.sqrt((s1[0] - s2[0])**2 + (s1[1] - s2[1])**2)
		return dist


	def computeShortestPath(self, G):
		while True:
			top_k1, top_k2 = self.U.topKey()
			start_k1, start_k2 = self.calculateKey(self.s_start)

			if [top_k1, top_k2] < [start_k1, start_k2] or \
				self.rhs[self.s_start] != self.g[self.s_start]:
				
				k_old = [top_k1, top_k2]
				u = self.U.pop()
				#print(f'u = {u}')
				self.visited.add(u)
				
				u_k1, u_k2 = self.calculateKey(u)
				if k_old < [u_k1, u_k2]:
					self.U.push(u, u_k1, u_k2)
				elif self.g[u] > self.rhs[u]:
					self.g[u] = self.rhs[u]
				else:
					self.g[s] = float('inf')
					self.updateVertex(u, G)

				u_succ = list(G.neighbors(u))
				for s in u_succ:
					#print(f's = {s}')
					self.updateVertex(s, G)

			else:
				break

	def extract_path(self, G):
		path = [self.s_start]
		s = self.s_start

		while True:
			g_list = {}
			s_succ = list(G.neighbors(s))
			for u in s_succ:
				g_list[u] = self.g[u]
			s = min(g_list, key=g_list.get)
			path.append(s)

			if s == self.s_goal:
				break

		return list(path)

occ_map_path = f'{cfg.PATH.OCC_MAP}/2t7WUuJeko7_0'
occupancy_map = np.load(f'{occ_map_path}/BEV_occupancy_map.npy', allow_pickle=True).item()['occupancy']

G = build_graph(occupancy_map)

start_coords = random.choice(list(G.nodes))
reachable_locs = list(nx.node_connected_component(G, start_coords))
end_coords = random.choice(reachable_locs)

lpa = DStarLite(start_coords, end_coords)
lpa.initialize(G)
lpa.computeShortestPath(G)

path = lpa.extract_path(G)

path = np.array(path) # N x 2

fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 5), dpi=125)
ax.imshow(occupancy_map, cmap='gray', vmin=0, vmax=1)
num_points = path.shape[0]
ax.scatter(path[:, 1], path[:, 0], c=range(num_points), cmap='viridis', s=np.linspace(4, 2, num=num_points)**2)
fig.tight_layout()
plt.show()