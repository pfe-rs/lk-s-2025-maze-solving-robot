import numpy as np
import core 

RADIUS = 100
KORAK = 0.1

def generisanjeUglova(n):
    return np.linspace(0, 2 * np.pi, n)

def pucanjeLasera(senzor, robot, mapa):

    y, x = robot["pozicija"]
    n = senzor["num_lasers"]

    udaljenostIUgao = []

    for ugao in generisanjeUglova(n):
        udaljenost = 0.0

        while udaljenost <= RADIUS:
            x0 = int(round(x + np.cos(ugao) * udaljenost))
            y0 = int(round(y - np.sin(ugao) * udaljenost))

            if not (0 <= x0 < mapa['celije'].shape[1] and 0 <= y0 < mapa['celije'].shape[0]): 
                break

            if mapa['celije'][y0, x0] == 1: 
                break

            udaljenost = udaljenost + KORAK

        udaljenostIUgao.append((udaljenost, ugao))

    return np.array(udaljenostIUgao)

def formiranjeSkena(senzor, robot, mapa):
    sken_matrix = np.zeros(mapa['celije'].shape)
    sken_hit = []
    sken_miss = []
    y, x = robot["pozicija"]
    n = senzor["num_lasers"]

    for udaljenost, ugao in pucanjeLasera(senzor, robot, mapa):
        if udaljenost > RADIUS: continue

        x0 = int(np.floor(x + np.cos(ugao) * udaljenost))
        y0 = int(np.floor(y - np.sin(ugao) * udaljenost))

        if not (0 <= x0 < mapa['celije'].shape[1] and 0 <= y0 < mapa['celije'].shape[0]): break 

        sken_matrix[y0, x0] = 1

    return {
        "sken_matrix": sken_matrix,
        "sken_hit": sken_hit,
        "sken_miss": sken_miss
    }

def detekcija_relativna(senzor: dict, robot: dict, mapa1: dict) -> np.ndarray:
    
    sken_data = formiranjeSkena(robot["pozicija"][0], robot["pozicija"][1], mapa1["celije"], n=360)  # n = broj uglova za laserski senzor

    centar_x, centar_y = int(robot["pozicija"][0]), int(robot["pozicija"][1])

    start_x = max(0, centar_x - 10)
    start_y = max(0, centar_y - 10)
    end_x = min(sken_data["sken_matrix"].shape[1], centar_x + 10)
    end_y = min(sken_data["sken_matrix"].shape[0], centar_y + 10)

    sken_20x20 = sken_data["sken_matrix"][start_y:end_y, start_x:end_x]

    return {
        "sken": sken_20x20,
        "hits": sken_data["sken_hit"],
        "misses": sken_data["sken_miss"]
    }

def detekcija_matrix(senzor: dict, robot: dict, mapa1: dict) -> np.ndarray:
    sken_data = {
        "sken_matrix": np.zeros(mapa1["celije"].shape),
        "sken_hit": [],
        "sken_miss": []
    }

    for i in range(mapa1["celije"].shape[0]): 
        for j in range(mapa1["celije"].shape[1]):
            if (i - robot["pozicija"][0]) ** 2 + (j - robot["pozicija"][1]) ** 2 < senzor["radius"] ** 2:
                sken_data["sken_matrix"][i, j] = mapa1["celije"][i, j]
                
              #  if mapa1["celije"][i, j] == 1:
               #     sken_data["sken_hit"].append((i, j))
               # else:
                #    sken_data["sken_miss"].append((i, j))
    return sken_data

    
