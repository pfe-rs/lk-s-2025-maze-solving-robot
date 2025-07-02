import pygame
import numpy as np
import detekcija
import random
import core
from kretanje import kretanje
import mapiranje

 ## TODO : izbrisati fajl posle
# tmp_linspace = np.linspace(0, 2 * np.pi, 100)
# ne koristimo ovo, ne znam zasto ga imamo


map_data = core.ucitaj_mapu("dvosobni_stan.pkl")
mapa = map_data["mapa"]

CELL = 5
WIDTH, HEIGHT = 120, 60
RES = SCREEN_WIDTH, SCREEN_HEIGHT = WIDTH * CELL, HEIGHT * CELL
#djelimo mapu na piksele
ROBOT_SPEED = 1
#sve stavljamo na crna

def nasumicna_pozicija_na_bijeloj_podlozi(mapa):
    while True:
        y = random.randint(0, mapa["celije"].shape[0] - 1)
        x = random.randint(0, mapa["celije"].shape[1] - 1)
        if mapa["celije"][y, x] == 0:
            return np.array([y, x])
        
cilj = nasumicna_pozicija_na_bijeloj_podlozi(mapa)
#np.array koristimo kada pravimo "pametne" liste, recimo ova lista moze da se sabere sa nekim koordinatama i ona ce da izbaci druge koordinate,
# a ne listu od 4 broja

# TODO: Staviti ostale promenljive u dict
robot = {
    "pozicija": nasumicna_pozicija_na_bijeloj_podlozi(mapa),
    "ugao": np.pi/4
    
}

mapaDict = {
    "celije": np.ones((HEIGHT, WIDTH)),
    "linije": []
}

#pygame
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT*2))
plot1 = pygame.Surface(RES)
plot2 = pygame.Surface(RES)


def draw():
    #plot 1
    for i in range(HEIGHT):
        for j in range(WIDTH):
            pygame.draw.rect(plot1, (0, 0, 0) if mapa[i,j] == 0 else (255, 255, 255),
                             (j*CELL, i*CELL, CELL, CELL))
    
    for robotPos in putanja:
        pygame.draw.circle(plot1, (150, 150, 255), robotPos[::-1]*CELL, 3)
    
    pygame.draw.circle(plot1, (255, 0, 0), robot["pozicija"][::-1]*CELL, 5)
    pygame.draw.circle(plot1, (0, 255, 0), cilj[::-1]*CELL, 5)
    
    #plot 2
    for i in range(HEIGHT):
        for j in range(WIDTH):
            pygame.draw.rect(plot2, (0, 0, 0) if interna_mapa[i,j] == 0 else (255, 255, 255),
                             (j*CELL, i*CELL, CELL, CELL))  
    
    screen.blit(plot1, (0, 0))
    screen.blit(plot2, (0, SCREEN_HEIGHT))
    pygame.display.update()

# primer
putanja = []
putanja.append(robot["pozicija"])

running = True
koraci = 200

senzor = core.senzor()
istorija_int_mapa = []
interni_robot = core.robot(np.array([0, 0]))
interna_mapa = core.grid_mapa(mapa["celije"], mapa["cilj"])

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            running = False
    if koraci<=0 or not running: continue
    koraci-=1

    sken = detekcija.detekcija(senzor, robot, mapa)
    interna_mapa["celije"] = mapiranje.mapiranje(interna_mapa, sken, robot)
    istorija_int_mapa.append(interna_mapa["celije"])
    robot["pozicija"], interni_robot["pozicija"] = kretanje.kretanje(interni_robot, robot, mapa["cilj"], interna_mapa)
    putanja.append(robot["pozicija"])
    draw()