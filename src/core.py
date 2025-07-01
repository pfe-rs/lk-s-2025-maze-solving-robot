import matplotlib.pyplot as plt
import numpy as np

tmp_linspace = np.linspace(0, 2 * np.pi, 100)

SCREEN_WIDTH, SCREEN_HEIGHT = 120, 60
ROBOT_SPEED = 1

mapa = np.ones((SCREEN_HEIGHT, SCREEN_WIDTH))
mapa[10:40, 10:40] = 0
mapa[10:40, 50:80] = 0
mapa[20:30, 40:50] = 0

interna_mapa = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH))

cilj = np.array([15, 70])

senzor_radius = 10

# TODO: Staviti ostale promenljive u dict
robot = {
    "pozicija": np.array([15, 15]),
    "ugao": np.pi/4
}

mapa = {
    "celije": np.ones((SCREEN_HEIGHT, SCREEN_WIDTH)),
    "linije": []
}

def detekcija(senzor_radius: float, robot: dict, mapa: np.ndarray) -> np.ndarray:
    sken = np.zeros(mapa.shape)
    for i in range(mapa.shape[0]):
        for j in range(mapa.shape[1]):
            if (i - robot["pozicija"][0]) ** 2 + (j - robot["pozicija"][1]) ** 2 < senzor_radius ** 2:
                sken[i][j] = mapa[i][j]
    return sken

def mapiranje(interna_mapa: np.ndarray, sken: np.ndarray) -> np.ndarray:
    return 1 - (1 - interna_mapa) * (1 - sken)

def kretanje(robot_pozicija: tuple[float, float], cilj: np.ndarray, interna_mapa: np.ndarray) -> tuple[float, float]:
    queue = [cilj.astype(int)]
    visited = np.zeros((interna_mapa.shape[0], interna_mapa.shape[1], 2))
    while len(queue) > 0:
        current_pos = queue.pop()
        for dir in np.array([(1, 0), (-1, 0), (0, 1), (0, -1)]):
            neighbor = current_pos + dir
            if 0 <= neighbor[0] < interna_mapa.shape[0] and 0 <= neighbor[1] < interna_mapa.shape[1]:
                if interna_mapa[neighbor[0]][neighbor[1]] == 0 and np.linalg.norm(visited[neighbor[0]][neighbor[1]]) == 0:
                    queue = [neighbor] + queue
                    visited[neighbor[0], neighbor[1]] = current_pos

    novi_cilj = visited[int(robot_pozicija[0]), int(robot_pozicija[1])]
    razlika = novi_cilj - robot_pozicija
    razlika = razlika / np.linalg.norm(razlika)
    pomeraj = razlika * ROBOT_SPEED
    return robot_pozicija + pomeraj


# primer
putanja = []
putanja.append(robot_pozicija)
for i in range(50):
    sken = detekcija(senzor_radius, robot_pozicija, mapa)
    interna_mapa = mapiranje(interna_mapa, sken)
    robot_pozicija = kretanje(robot_pozicija, cilj, interna_mapa)
    print(i, robot_pozicija)
    putanja.append(robot_pozicija)

putanja = np.array(putanja)

# TODO: Zameniti u pygame
fig, (ax1, ax2) = plt.subplots(2, 1)
ax1.plot(robot_pozicija[1], robot_pozicija[0], "o")
ax1.plot(cilj[1], cilj[0], "o")
ax1.plot(putanja, "--")
ax1.plot(robot_pozicija[1] + senzor_radius * np.sin(tmp_linspace), robot_pozicija[0] + senzor_radius * np.cos(tmp_linspace), "r")
ax1.imshow(mapa, cmap="gray")
ax2.imshow(interna_mapa)
plt.show()
