from yacs.config import CfgNode as CN 

_C = CN()

_C.PATH = CN()
_C.PATH.OCC_MAP = 'data/occ_maps'


_C.GENERAL = CN()
_C.GENERAL.RANDOM_SEED = 5


_C.MAP = CN()
_C.MAP.FREE_VAL = 1
_C.MAP.COLLISION_VAL = 0