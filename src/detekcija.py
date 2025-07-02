import numpy as np

RADIUS = 10
KORAK = 0.1

def generisanjeUglova(n):
    return np.linspace(0, 2 * np.pi, n)

def pucanjeLasera(x, y, mapa, n):
    udaljenostIUgao = []

    for ugao in generisanjeUglova(n):
        udaljenost = 0.0

        while udaljenost <= RADIUS:
            x0 = int(round(x + np.cos(ugao) * udaljenost))
            y0 = int(round(y - np.sin(ugao) * udaljenost))

            if not (0 <= x0 < mapa.shape[1] and 0 <= y0 < mapa.shape[0]): 
                break

            if mapa[y0, x0] == 1: 
                break

            udaljenost = udaljenost + KORAK

        udaljenostIUgao.append((udaljenost, ugao))

    return np.array(udaljenostIUgao)

def formiranjeSkena(x, y, mapa, n):
    sken = np.zeros(mapa.shape)

    for udaljenost, ugao in pucanjeLasera(x, y, mapa, n):
        if udaljenost > RADIUS: continue

        x0 = int(np.floor(x + np.cos(ugao) * udaljenost))
        y0 = int(np.floor(y - np.sin(ugao) * udaljenost))

        if not (0 <= x0 < mapa.shape[1] and 0 <= y0 < mapa.shape[0]): break 

        sken[y0, x0] = 1

    return sken

def detekcija(senzor_radius: float, robot: dict, mapa1: dict) -> np.ndarray:
    
    sken = formiranjeSkena(robot["pozicija"][0], robot["pozicija"][1], mapa1["celije"], n=360)  # n = broj uglova za laserski senzor

    centar_x, centar_y = int(robot["pozicija"][0]), int(robot["pozicija"][1])

    start_x = max(0, centar_x - 10)
    start_y = max(0, centar_y - 10)
    end_x = min(sken.shape[1], centar_x + 10)
    end_y = min(sken.shape[0], centar_y + 10)

    sken_20x20 = sken[start_y:end_y, start_x:end_x]

    return sken_20x20