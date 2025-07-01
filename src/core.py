import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt
import pygame

pygame.init()

SCREEN_WIDTH, SCREEN_HEIGHT = 120, 60
screen = pygame.display.set_mode((SCREEN_WIDTH*10, SCREEN_HEIGHT*20))
pygame.display.set_caption("slam brute force")

senzor_radius = 10
cilj = np.array([15, 70])
tmp_linspace = np.linspace(0, 2 * np.pi, 100)

robot = {
    "pozicija": np.array([15, 15]),
    "ugao": np.pi/4,
    "robot_brzina": 1,
}

mapa = {
    "celije": np.ones((SCREEN_HEIGHT, SCREEN_WIDTH)),
    "linije": []
}

interna_mapa = {
    "celije": np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH)),
    "linije": [] 
}

mapa["celije"][10:40, 10:40] = 0
mapa["celije"][10:40, 50:80] = 0
mapa["celije"][20:30, 40:50] = 0

def detekcija(senzor_radius: float, robot: dict, mapa: dict) -> np.ndarray:
    sken = np.zeros(mapa["celije"].shape)
    for i in range(mapa["celije"].shape[0]):
        for j in range(mapa["celije"].shape[1]):
            if (i - robot["pozicija"][0]) ** 2 + (j - robot["pozicija"][1]) ** 2 < senzor_radius ** 2:
                sken[i][j] = mapa["celije"][i][j]
    return sken

def mapiranje(interna_mapa: dict, sken: np.ndarray) -> np.ndarray:
    return 1 - (1 - interna_mapa["celije"]) * (1 - sken)

def kretanje(robot: dict, pozicija: Tuple[float, float], cilj: np.ndarray, interna_mapa: dict) -> Tuple[float, float]:
    queue = [cilj.astype(int)]
    visited = np.zeros((interna_mapa["celije"].shape[0], interna_mapa["celije"].shape[1], 2))
    while len(queue) > 0:
        current_pos = queue.pop()
        for dir in np.array([(1, 0), (-1, 0), (0, 1), (0, -1)]):
            neighbor = current_pos + dir
            if 0 <= neighbor[0] < interna_mapa["celije"].shape[0] and 0 <= neighbor[1] < interna_mapa["celije"].shape[1]:
                if interna_mapa["celije"][neighbor[0]][neighbor[1]] == 0 and np.linalg.norm(visited[neighbor[0]][neighbor[1]]) == 0:
                    queue = [neighbor] + queue
                    visited[neighbor[0], neighbor[1]] = current_pos

    novi_cilj = visited[int(robot["pozicija"][0]), int(robot["pozicija"][1])]
    razlika = novi_cilj - robot["pozicija"]
    razlika = razlika / np.linalg.norm(razlika)
    pomeraj = razlika * robot["robot_brzina"]
    return robot["pozicija"] + pomeraj


def prikazi_skeniranje(screen, robot:dict, senzor_radius, tmp_linspace):
    pygame.draw.circle(screen, (255, 0, 0), 
                       (int(robot["pozicija"][1] * 10), int(robot["pozicija"][0] * 10)), 
                       senzor_radius * 10, width=1)

    for angle in tmp_linspace:
        x_end = robot["pozicija"][1] + senzor_radius * np.sin(angle)
        y_end = robot["pozicija"][0] + senzor_radius * np.cos(angle)
        pygame.draw.line(screen, (255, 0, 0),
                         (int(robot["pozicija"][1] * 10), int(robot["pozicija"][0] * 10)),
                         (int(x_end * 10), int(y_end * 10)), 1)

running = True
putanja = []
putanja.append(robot["pozicija"])
clock = pygame.time.Clock()

# TODO : razdvojiti simulaciju od vizueliacije
# TODO : napraviti funkciju za simulaciju (pocetni polozaj robota, cilj, mapa) -> putanja, listu internih mapa
# TODO : napraviti funkciju za vizuelizaciju (mapa, putanja, lista internih mapa)


while running:
    #screen.fill((255, 255, 255))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    sken = detekcija(senzor_radius, robot, mapa)
    interna_mapa["celije"] = mapiranje(interna_mapa, sken)
    robot["pozicija"] = kretanje(robot, robot["pozicija"], cilj, interna_mapa)
    putanja.append(robot["pozicija"])

    # obicna mapa
    for i in range(SCREEN_HEIGHT):
        for j in range(SCREEN_WIDTH):
           color = (0, 0, 0) if mapa["celije"][i, j] == 0 else (255, 255, 255)
           pygame.draw.rect(screen, color, (j * 10, i * 10, 10, 10))

    # interna mapa
    for i in range(SCREEN_HEIGHT):
       for j in range(SCREEN_WIDTH):
            color = (0, 0, 0) if interna_mapa["celije"][i, j] == 0 else (255, 255, 255)
            pygame.draw.rect(screen, color, (j * 10, i * 10 + SCREEN_HEIGHT * 10, 10, 10))

    # robot i cilj
    pygame.draw.circle(screen, (255, 0, 0), (int(robot["pozicija"][1] * 10), int(robot["pozicija"][0] * 10)), 5)
    pygame.draw.circle(screen, (0, 255, 0), (int(cilj[1] * 10), int(cilj[0] * 10)), 5)

    for p in putanja:
       pygame.draw.circle(screen, (0, 0, 255), (int(p[1] * 10), int(p[0] * 10)), 2)

   # prikazi_matplotlib(robot, cilj, putanja, senzor_radius, tmp_linspace, mapa, interna_mapa)

    pygame.display.flip()
    clock.tick(30)

pygame.quit()