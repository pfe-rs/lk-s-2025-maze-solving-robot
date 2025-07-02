import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt
import pygame
#import maze 
import pickle
import random

SCREEN_WIDTH, SCREEN_HEIGHT = 120, 60

senzor_radius = 10
cilj = np.array([15, 70])
tmp_linspace = np.linspace(0, 2 * np.pi, 100)

def ucitaj_mapu(fajl):
    with open(fajl, "rb") as f:
        mapa = pickle.load(f)
    return {"celije": mapa["mapa"]}

def generate_map():
    # 1 je slobodno polje
    mapa = np.ones((SCREEN_HEIGHT, SCREEN_WIDTH))  
    
    w = random.randint(5, 20)
    h = random.randint(5, 20)

    if w > SCREEN_WIDTH or h > SCREEN_HEIGHT:
        pass 

    else:
        #slucajna pozicija za sobu u mapi
        x = random.randint(0, SCREEN_WIDTH - w) 
        y = random.randint(0, SCREEN_HEIGHT - h)
        
        mapa[y:y+h, x:x+w] = 0  # postavlja se 0 gde je soba

    return mapa

def generisi_mapu():
    map_data = []
    
    for i in range(5): 
        matrix = generate_map() # generisem mapu sa sobama

        # random pocetna i krajnja tacka u mapi
        start_point = np.array([np.random.randint(0, SCREEN_HEIGHT), np.random.randint(0, SCREEN_WIDTH)])
        end_point = np.array([np.random.randint(0, SCREEN_HEIGHT), np.random.randint(0, SCREEN_WIDTH)])

        # cuvam mapu sa pocetnim i krajnjim tackama za robota
        map_data.append({
            'mapa': matrix,
            'start': start_point.tolist(),
            'end': end_point.tolist()
        })

        # cuvanje u fjl
        with open(f"mapa_{i+1}.pkl", "wb") as f:
            pickle.dump(map_data[-1], f)
        print(f"Mapa {i+1} je sačuvana u fajl mapa_{i+1}.pkl.")

    return map_data


mape = generisi_mapu()
#print(f"screen width:{SCREEN_WIDTH}, height{SCREEN_HEIGHT}, w {W}, h {h}")
mapa1 = ucitaj_mapu("mapa_1.pkl")

robot = {
    "pozicija": np.array([15, 15]),
    "ugao": np.pi/4,
    "robot_brzina": 1,
}

interni_robot = {
    "pozicija": np.array([0, 0]),
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

#mapa["celije"][10:40, 10:40] = 0
#mapa["celije"][10:40, 50:80] = 0
#mapa["celije"][20:30, 40:50] = 0


def mapiranje(interna_mapa: dict, sken: np.ndarray, robot: dict) -> np.ndarray:
    return 1 - (1 - interna_mapa["celije"]) * (1 - sken)

def kretanje(interni_robot: dict, pozicija: Tuple[float, float], cilj: np.ndarray, interna_mapa: dict) -> Tuple[float, float]:
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
    return robot["pozicija"] + pomeraj, interni_robot["pozicija"] + pomeraj


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

def simulacija(robot: dict, pozicija: Tuple[float, float], cilj: np.ndarray, mapa1: dict):

    for i in range(50):
        print(senzor_radius)
        print(robot)
        sken = detekcija(senzor_radius, robot, mapa1)
        interna_mapa["celije"] = mapiranje(interna_mapa, sken, robot)
        istorija_int_mapa.append(interna_mapa["celije"])
        robot["pozicija"], interni_robot["pozicija"] = kretanje(robot, robot["pozicija"], cilj, interna_mapa)
        print(i, robot["pozicija"])
        putanja.append(robot["pozicija"])

    return putanja, istorija_int_mapa

def vizuelizacija(mapa: dict, putanja: list, istorija_int_mapa: list):
    pygame.init()
    
    screen = pygame.display.set_mode((SCREEN_WIDTH*10, SCREEN_HEIGHT*20))
    pygame.display.set_caption("slam brute force")

    running = True
    clock = pygame.time.Clock()
        
    while running:
        #screen.fill((255, 255, 255))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        
        # original mapa
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

putanja = []
istorija_int_mapa = []

putanja, istorija_int_mapa = simulacija(robot, robot["pozicija"], cilj, mapa)

#putanja.append(robot["pozicija"])

vizuelizacija(mapa, putanja, istorija_int_mapa)

# DONE : razdvojiti simulaciju od vizuelizacije
# DONE : napraviti funkciju za simulaciju (pocetni polozaj robota, cilj, mapa) -> putanja, listu internih mapa
# DONE : napraviti funkciju za vizuelizaciju (mapa, putanja, lista internih mapa)

# TODO : dataset(sacuvane u fajlu) generisanih mapa, sa pocetnom i krajnjom tackom {pickle}
# TODO : interna reprezentacija robota, sken treba da je matrica dimenzija [2*sken radius][2*sken radius], promeniti mapiranje da radi sa tom matricom skena, 
# TODO : detekciju izmeniti, gledati da sken ne bude cela matrica vec samo deo koji je skeniran tj dimenzija [2*sken radius][2*sken radius], traziti odakle dokle ide for petlja itd


# TODO : detekcija => koristim pravu poziciju robota, sken se ogranicava na 20*20 umesto na celu mapu
# TODO : mapiranje => prosledjujem pravog robota, siftujem sken za +- radijus skena 