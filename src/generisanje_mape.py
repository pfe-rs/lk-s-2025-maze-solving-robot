import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt
import pygame
#import maze 
import pickle
import random
from generisanje_mape import mapa


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
