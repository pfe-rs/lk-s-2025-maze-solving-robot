import numpy as np
import random
import pickle
import core


def nasumicna_pozicija_na_bijeloj_podlozi(mapa):
    while True:
        y = random.randint(0, mapa.shape[0] - 1)
        x = random.randint(0, mapa.shape[1] - 1)
        if mapa[y, x] == 0:
            return np.array([y, x])

WIDTH=120
HEIGHT=60


broj_soba = 4
min_sirina_sobe = 30
max_sirina_sobe = 30
min_visina_sobe = 20
max_visina_sobe = 20

mapa = np.ones((HEIGHT, WIDTH), dtype=int)
 #pravimo matricu punu zidova

sobe = [] # Lista soba, svaka soba ce imati (x, y, sirina, visina), a izlaz ce da bude pravougaonik

def sobe_se_preklapaju(soba1, soba2):
    x1, y1, w1, h1 = soba1 # x1, y1 su koordinate gornjeg lijevog ugla sobe, w1 i h1 su dimenzije sobe 
    x2, y2, w2, h2 = soba2 # isto to samo za drugu sobu 
    # Provjera da li se pravougaonici ne dodiruju 
    if x1 + w1 + 1 < x2:
        return False
    if x2 + w2 + 1 < x1:
        return False
    if y1 + h1 + 1 < y2:
        return False
    if y2 + h2 + 1 < y1:
        return False
    #ovdje provlacimo svaki sa svakim da provjerimo da li se sobe preklapaju 
    return True

while len(sobe) < broj_soba:
    sirina_sobe = random.randint(min_sirina_sobe, max_sirina_sobe)
    visina_sobe = random.randint(min_visina_sobe, max_visina_sobe)
    x = random.randint(0, WIDTH - sirina_sobe - 1)
    y = random.randint(0, HEIGHT - visina_sobe - 1)
    

    novaSoba = (x, y, sirina_sobe, visina_sobe)

    # Provjeravamo da li se nova soba preklapa sa postojećim sobama
    preklapanje = False
    for soba in sobe:
        if sobe_se_preklapaju(novaSoba, soba):
            preklapanje = True
            break

    # Ako nema preklapanja, dodaj sobu u listu i iscrtaj u matricu
    if not preklapanje:
        sobe.append(novaSoba)
        # Postavi prostor sobe na 0 (dio kojim robot koraca)
        x, y, w, h = novaSoba
        for i in range(y, y + h):
            for j in range(x, x + w):
                mapa[i][j] = 0

# Funkcija za dobijanje centra sobe
def centar_sobe(soba):
    x, y, w, h = soba
    return (x + w//2, y + h//2)

# Povezivanje soba hodnicima u obliku slovq L
SIRINA_ZIDA=10
SA_STRANA=SIRINA_ZIDA//2
for i in range(1, len(sobe)):
    (x1, y1) = centar_sobe(sobe[i-1])
    (x2, y2) = centar_sobe(sobe[i])

    # Prvo horizontalno iskopaj hodnik između soba
    if x1 < x2:
        for x in range(x1, x2+1):
            for dy in range(-SA_STRANA, SA_STRANA+1):
                y=y1+dy
                if 0<y<HEIGHT:
                    mapa[y][x] = 0
    else:
        for x in range(x2, x1+ 1):
            for dy in range(-SA_STRANA, SA_STRANA+1):
                y=y1+dy
                if 0<y<HEIGHT:
                    mapa[y][x] = 0

    if y1 < y2:
        for y in range(y1, y2 + 1):
            for dx in range(-SA_STRANA, SA_STRANA+1):
                x=x1+dx
                if 0<x<WIDTH:
                    mapa[y][x] = 0
    else:
        for y in range(y2, y1 + 1):
            for dx in range (-SA_STRANA,SA_STRANA+1):
                x=x1+dx
                if 0<x<WIDTH:
                    mapa[y][x] = 0


print(mapa)

start_point = nasumicna_pozicija_na_bijeloj_podlozi(mapa)
end_point = nasumicna_pozicija_na_bijeloj_podlozi(mapa)

with open("mapa8.pkl", "wb") as f: 
    pickle.dump({
            'mapa': core.grid_mapa(mapa, end_point),
            'start': start_point,
            'end': end_point
        }, f) #pretvara mapu u niz bajtova i zapisuje u fajl f 
    #cuvanje mape u pickle file 


