import numpy as np

# TODO: Prebacite sve ovo u detekcija.py

# TODO: Umesto da radius bude deo laser.py (ili detekcija.py), treba da postane argument funkcije pucanje_lasera
RADIUS = 10

# TODO: korak definisite u funkciji koracanje_lasera
KORAK = 0.1

def generisanjeUglova(n):
    return np.linspace(0, 2 * np.pi, n)

# TODO: Umesto x, y, mapa, n, ulazi treba da budu robot, prava_mapa, senzor
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


# TODO: Umesto x, y, mapa, n, ulazi treba da budu robot, prava_mapa, senzor
def formiranjeSkena(x, y, mapa, n):

    sken = np.zeros(mapa.shape)

    for udaljenost, ugao in pucanjeLasera(x, y, mapa, n):
        if udaljenost > RADIUS: continue

        #koristimo np.floor kako se negativne vrednosti ne bi zaokruzile
        #na nulu (u slucaju da laser ode van mape), vec na -1
        x0 = int(np.floor(x + np.cos(ugao) * udaljenost))
        y0 = int(np.floor(y - np.sin(ugao) * udaljenost))

        if not (0 <= x0 < mapa.shape[1] and 0 <= y0 < mapa.shape[0]): break 

        sken[y0, x0] = 1

    return sken