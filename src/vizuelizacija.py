import pygame
import numpy as np
import pickle
import core

def vizuelizacija(mapa: dict, cilj: np.ndarray, putanja: list, istorija_int_mapa: list):
    pygame.init()
    
    mapa = mapa["celije"]
    
    cell_size = 5

    screen = pygame.display.set_mode((mapa.shape[1] * cell_size, mapa.shape[0] * cell_size * 2))
    pygame.display.set_caption("slam brute force")

    running = True
    clock = pygame.time.Clock()
        
    frame = 0
    while running:
        #screen.fill((255, 255, 255))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        
        # original mapa
        for i in range(mapa.shape[0]):
            for j in range(mapa.shape[1]):
                color = (0, 0, 0) if mapa[i, j] == 0 else (255, 255, 255)
                pygame.draw.rect(screen, color, (j * cell_size, i * cell_size, cell_size, cell_size))

        # interna mapa
        for i in range(mapa.shape[0]):
            for j in range(mapa.shape[1]):
                color = (0, 0, 0) if istorija_int_mapa[frame][i, j] == 0 else (255, 255, 255)
                pygame.draw.rect(screen, color, (j * cell_size, i * cell_size + mapa.shape[1] * cell_size, cell_size, cell_size))

        # robot i cilj
        pygame.draw.circle(screen, (0, 255, 0), (int(cilj[1] * cell_size), int(cilj[0] * cell_size)), 5)

        for p in putanja[:frame]:
            pygame.draw.circle(screen, (0, 0, 255), (int(p[1] * cell_size), int(p[0] * cell_size)), 2)

        
        pygame.draw.circle(screen, (255, 0, 0), (int(putanja[-1][1] * cell_size), int(putanja[-1][0] * cell_size)), 5)
    

        # prikazi_matplotlib(robot, cilj, putanja, senzor_radius, tmp_linspace, mapa, interna_mapa)

        pygame.display.flip()
        clock.tick(100)

        frame += 1
        if frame >= len(putanja):
            frame = len(putanja) - 1

    pygame.quit()

if __name__ == "__main__":
    
    mapa = {
        "celije": np.ones((60, 120)),
        "linije": []
    }
    
    mapa["celije"][10:40, 10:40] = 0
    mapa["celije"][10:40, 50:80] = 0
    mapa["celije"][20:30, 40:50] = 0

    cilj = np.array([15, 70])

    map_data = core.ucitaj_mapu("dvosobni_stan.pkl")
    mapa = map_data["mapa"]
    cilj = map_data["end"]

    with open("simulacija_rezultati.pkl", "rb") as f:
        rez = pickle.load(f)
    
    putanja, istorija_int_mapa = rez["putanja"], rez["istorija_int_mapa"]

    vizuelizacija(mapa, cilj, putanja, istorija_int_mapa)