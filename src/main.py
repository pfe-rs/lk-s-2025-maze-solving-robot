import pygame
import numpy as np
from laser import formiranjeSkena
import random
from mapa import mapa
from idi import *

CELL = 5
WIDTH, HEIGHT = 120, 60
RES = SCREEN_WIDTH, SCREEN_HEIGHT = WIDTH * CELL, HEIGHT * CELL
#djelimo mapu na piksele
ROBOT_SPEED = 1

interna_mapa = np.zeros((300, 300))
#sve stavljamo na crna

def nasumicna_pozicija_na_bijeloj_podlozi(mapa):
    while True:
        y = random.randint(0, mapa.shape[0] - 1)
        x = random.randint(0, mapa.shape[1] - 1)
        if mapa[y, x] == 0:
            return np.array([y, x])
        
cilj = nasumicna_pozicija_na_bijeloj_podlozi(mapa)

robot = {
    "pozicija": nasumicna_pozicija_na_bijeloj_podlozi(mapa),
    "ugao": np.pi/4,
    "interPoz": np.array(np.array(interna_mapa.shape)//2)
}
robot["interCilj"] = np.array([robot["interPoz"][0]+cilj[0]-robot["pozicija"][0],
                      robot["interPoz"][1]+cilj[1]-robot["pozicija"][1]])

mapaDict = {
    "celije": np.ones((HEIGHT, WIDTH)),
    "linije": []
}

def mapiranje(interna_mapa: np.ndarray, sken: np.ndarray) -> np.ndarray:
    return 1 -(1-sken) * (1-interna_mapa)




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
            y = robot["interPoz"][0]-HEIGHT//2+i
            x = robot["interPoz"][1]-WIDTH//2+j
            pygame.draw.rect(plot2, (0, 0, 0) if interna_mapa[y, x] == 0 else (255, 255, 255),
                             (j*CELL, i*CELL, CELL, CELL))  
    
    screen.blit(plot1, (0, 0))
    screen.blit(plot2, (0, SCREEN_HEIGHT))
    pygame.display.update()

# primer
putanja = []

running = True
koraci = 100
draw()
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            running = False
    if koraci<=0 or not running: continue
    koraci-=1
    putanja.append(robot["pozicija"].copy())
    
    sken = formiranjeSkena(robot, mapa, interna_mapa, 50)
    interna_mapa = mapiranje(interna_mapa, sken)
    pomeraj = kretanje(robot["interPoz"], robot["interCilj"], interna_mapa)
    robot["pozicija"] += pomeraj
    robot["interPoz"] += pomeraj
    print(koraci, robot["pozicija"])
    draw()