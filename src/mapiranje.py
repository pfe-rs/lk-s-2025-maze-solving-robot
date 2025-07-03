from core import *


def mapiranje(interna_mapa: dict, sken_data: dict, robot: dict) -> np.ndarray:
    sken_matrix = sken_data["sken_matrix"]
    return 1 - (1 - interna_mapa["celije"]) * (1 - sken_matrix)
