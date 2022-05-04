import numpy as np 
import matplotlib.pyplot as plt 
from config import _C as cfg
import networkx as nx
import random
from my_LPAstar import LPA 
from utils import build_graph_full

random.seed(cfg.GENERAL.RANDOM_SEED)

NUM_RUN = 10
lst_ve, lst_va = [], []

scene_dict = {}
scene_dict['2t7WUuJeko7_0'] = {}
scene_dict['2t7WUuJeko7_0']['start'] = (23, 21)
scene_dict['2t7WUuJeko7_0']['goal'] = (76, 109)


for scene_name in list(scene_dict.keys()):
	print(f'----------------- {scene_name} --------------------')
	start_coords = scene_dict[scene_name]['start']
	end_coords = scene_dict[scene_name]['goal']


	#============================= load occ map ===========================
	occ_map_path = f'{cfg.PATH.OCC_MAP}/{scene_name}'
	occ_map = np.load(f'{occ_map_path}/BEV_occupancy_map.npy', allow_pickle=True).item()['occupancy']


	for run in range(NUM_RUN):
		occupancy_map = occ_map.copy()
		G = build_graph_full(occupancy_map)

		lpa = LPA(start_coords, end_coords)
		lpa.initialize(G)
		lpa.computeShortestPath(G)

		#================================ generate obstacles ==============================
		while True:
			num_nodes = len(G.nodes)
			obstacles = random.choices(list(G.nodes), k=int(num_nodes * .01))
			changed_occupancy_map = occupancy_map.copy()
			for obs in obstacles:
				changed_occupancy_map[obs] = cfg.MAP.COLLISION_VAL
			#changed_occupancy_map[np.array(obstacles).transpose()] = cfg.MAP.COLLISION_VAL

			G_prime = build_graph_full(changed_occupancy_map)

			try:
				n = nx.shortest_path_length(G_prime, start_coords, end_coords)
				print('still navigable')
				break
			except nx.NetworkXNoPath:
				print('no path')


		lpa.reset()
		for obs in obstacles:
			u_succ = list(G_prime.neighbors(obs))
			for u in u_succ:
				if u in G_prime.nodes:
					lpa.updateVertex(u, G_prime)

		#print(f'recompute path ...')
		lpa.computeShortestPath(G_prime)
		#print(f'done')
		#path = lpa.extract_path(G_prime)
		#path = np.array(path) # N x 2
		#print(f'len(path) = {len(path)}')

		print(f've = {len(lpa.visited)}')
		print(f'va = {lpa.count_va}')

		lst_ve.append(len(lpa.visited))
		lst_va.append(lpa.count_va)


lst_ve = np.array(lst_ve)
lst_va = np.array(lst_va)
print(f'avg_ve = {np.mean(lst_ve)}, avg_va = {np.mean(lst_va)}')