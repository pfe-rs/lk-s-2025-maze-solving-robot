from core import *


def mapiranje(interna_mapa: dict, sken: np.ndarray, robot: dict) -> np.ndarray:
    return 1 - (1 - interna_mapa["celije"]) * (1 - sken)
