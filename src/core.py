import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt
import pygame
#import maze 
import pickle
import random

"""
Ovde stavljam generalne komentare za organizaciju svega.
1. Napravite sledece fajlove:
    a) detekcija.py - Ovde ubacujete detekcija() funkciju i sve sto je u laser.py
    b) mapiranje.py - Ovde cete pisati sve funkcije koje su potrebne za obradu linija (za sada samo mapiranje() funkciju)
    c) kretanje.py - Ovde cete pisati sve vezano za navigaciju, BFS, A* i naravno kretanje() funkciju.
    d) simulacija.py - Ovde importujete detekcija.py, mapiranje.py i kretanje.py, ucitate mapu iz pickle-a i pokrenete ih u nekoj petlji i sacuvate rezultat u pickle.
    e) vizuelizacija.py - Ovde ne importujete nista od prethodnog, vec samo ucitate pickle fajl koji generise simulacija i prikazete ga (kao animacija ili slika, itd.)
    f) (Opcionalno) generisanje_mape.py - Ovde stavite sve vezano za generisanje mape i cuvanje iste u pickle fajl.
"""

# TODO: Ovo vam nece trebati kada implementirate ucitavanje mape iz pickla
# Trebace vam samo neki CELL_SIZE parametar kojim podesavate koliko velike celije ce biti.
SCREEN_WIDTH, SCREEN_HEIGHT = 120, 60

# TODO: Senzor pretvorite u dict. Na primer nesto poput ovoga:
# {
#     "radius" = 10, # koliko daleko moze laser da vidi.
#     "num_lasers" = 90 # koliko lasera ima senzor
# }
senzor_radius = 10

# TODO: Cilj treba da bude deo pickla za mapu, tako da vam ne treba ovde.
cilj = np.array([15, 70])

# TODO: Ovo se koristilo samo za iscrtavanje kruga u matplotlib-u. Vise vam ne treba u pygame-u
tmp_linspace = np.linspace(0, 2 * np.pi, 100)

# TODO: Prebaciti u simulacija.py
def ucitaj_mapu(fajl):
    with open(fajl, "rb") as f:
        mapa = pickle.load(f)
    return {"celije": mapa["mapa"]}

# TODO: Prebaciti u generisanje_mape.py
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


# TODO: prebaciti u generisanje_mape.py
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

# TODO: Ovo vam vise ne treba.
mape = generisi_mapu()
#print(f"screen width:{SCREEN_WIDTH}, height{SCREEN_HEIGHT}, w {W}, h {h}")

# TODO: Prebaciti u simulacija.py
mapa1 = ucitaj_mapu("mapa_1.pkl")

# TODO: Napravite funkciju robot(pozicija, ugao, brzina) koji vam vraca ovaj dict.
# To treba da ostane u core.py i da ga posle koristite u simulacija.py preko import-a
robot = {
    "pozicija": np.array([15, 15]),
    "ugao": np.pi/4,
    "brzina": 1,
}

interni_robot = {
    "pozicija": np.array([0, 0]),
    "ugao": np.pi/4,
    "robot_brzina": 1,
}

# TODO: Napravite funkcije koje prave ove dict-ove, jednu za mapu jednu za internu mapu.
# TODO: Mapu preimenujte u prava_mapa, da bi bilo jasnije o kojoj mapi se radi. Takodje, u pravoj mapi ne treba da imate "linije".
# TODO: Interna mapa treba da bude pretstavljena samo linijama. Za sada ostavite da ima i celije, ali se krecemo ka tome da nema celije, vec samo linije.
# TODO: Ubacite "cilj" u obe mape, kako bi BFS i A* znali gde treba da se ide.
mapa = {
    "celije": np.ones((SCREEN_HEIGHT, SCREEN_WIDTH)),
    "linije": []
}

interna_mapa = {
    "celije": np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH)),
    "linije": [] 
}

# TODO: Ovu mapu sacuvajte u mapa_0.pkl
mapa["celije"][10:40, 10:40] = 0
mapa["celije"][10:40, 50:80] = 0
mapa["celije"][20:30, 40:50] = 0

# TODO: Pomeriti u detekcija.py
# TODO: Zameniti senzor_radius sa senzor koji je dict.
# TODO: Promenite da sken bude dict: {"sken_matrix": <ovo sto trenutno imate kao sken>, "sken_hit": <laseri koji su udarili u zid>, "sken_miss": <laseri koji nisu udarili u zid>}
# TODO: Preimenujte ovu funkciju u detekcija_matrix, koja vraca sken dict gde "sken_hit" i "sken_miss" su prazne liste, a samo "sken_matrix" ima rezultat.
# TODO: Dodajte detekcija_laser koja prima iste argumente kao detekcija_matrix, ali vraca rezultat pokretanja funkcija koje su trenutno u laser.py
def detekcija(senzor_radius: float, robot: dict, mapa1: dict) -> np.ndarray:
    sken = np.zeros(mapa1["celije"].shape)
    for i in range(mapa1["celije"].shape[0]): 
        for j in range(mapa1["celije"].shape[1]):
            if (i - robot["pozicija"][0]) ** 2 + (j - robot["pozicija"][1]) ** 2 < senzor_radius ** 2:
                sken[i, j] = mapa1["celije"][i, j]
    return sken

# TODO: Pomeriti mapiranje() u mapiranje.py
# TODO: Promenite da mapiranje vraca dict koji je interna mapa, umesto samo celije. Na taj nacin ce lakse biti da predjemo na linijsku reprezentaciju mape
# TODO: Preimenujte ovu funkciju u mapiranje_matrix
# TODO: Dodajte mapiranje_lines koja ima isti potpis kao mapiranje_matrix, ali vraca dict za internu mapu u kojoj se nalazi "linije" popunjene.
def mapiranje(interna_mapa: dict, sken: np.ndarray, robot: dict) -> np.ndarray:
    return 1 - (1 - interna_mapa["celije"]) * (1 - sken)

# TODO: Pomeriti kretanje() u kretanje.py
# TODO: Umesto da se vraca tuple[float, float], treba da se vraca tuple[dict, dict], gde je prvi dict pravi_robot, a drugi interni_robot.
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


# TODO: Ovo prebacite u vizuelizacija.py
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

# TODO: Ovo prebacite u simulacija.py
# TODO: Ne treba vam pozicija argument.
# TODO: Cilj ce biti u sastavu mape, tako da vam ne treba kao argument.
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

# TODO: Pomerite u vizuelizacija.py
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

# TODO: Ovo vam vise ne treba.
putanja = []
istorija_int_mapa = []

# TODO: Ovo pomerite u simulacija.py
# TODO: Dodajte cuvanje putanje i istorije internih mapa u pickle.
putanja, istorija_int_mapa = simulacija(robot, robot["pozicija"], cilj, mapa)

#putanja.append(robot["pozicija"])

# TODO: Ovo pomerite u vizuelizacija.py
# TODO: Umesto koriscenja izracunatih vrednosti, dodajte ucitavanje rezultata iz pickle-a koje su generisane u simulacija.py
vizuelizacija(mapa, putanja, istorija_int_mapa)

# DONE : razdvojiti simulaciju od vizuelizacije
# DONE : napraviti funkciju za simulaciju (pocetni polozaj robota, cilj, mapa) -> putanja, listu internih mapa
# DONE : napraviti funkciju za vizuelizaciju (mapa, putanja, lista internih mapa)

# TODO : dataset(sacuvane u fajlu) generisanih mapa, sa pocetnom i krajnjom tackom {pickle}
# TODO : interna reprezentacija robota, sken treba da je matrica dimenzija [2*sken radius][2*sken radius], promeniti mapiranje da radi sa tom matricom skena, 
# TODO : detekciju izmeniti, gledati da sken ne bude cela matrica vec samo deo koji je skeniran tj dimenzija [2*sken radius][2*sken radius], traziti odakle dokle ide for petlja itd


# TODO : detekcija => koristim pravu poziciju robota, sken se ogranicava na 20*20 umesto na celu mapu
# TODO : mapiranje => prosledjujem pravog robota, siftujem sken za +- radijus skena 