import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt
import pygame
#import maze 
import pickle
import random
import core

SCREEN_HEIGHT = 60
SCREEN_WIDTH = 120

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

def generisi_mape():
    
    for i in range(5): 
        matrix = generate_map() # generisem mapu sa sobama

        # random pocetna i krajnja tacka u mapi
        start_point = np.array([np.random.randint(0, SCREEN_HEIGHT), np.random.randint(0, SCREEN_WIDTH)])
        end_point = np.array([np.random.randint(0, SCREEN_HEIGHT), np.random.randint(0, SCREEN_WIDTH)])

        # cuvam mapu sa pocetnim i krajnjim tackama za robota
        map_data = {
            'mapa': core.grid_mapa(matrix),
            'start': start_point,
            'end': end_point
        }

        # cuvanje u fjl
        with open(f"mapa_{i+1}.pkl", "wb") as f:
            pickle.dump(map_data, f)
        print(f"Mapa {i+1} je sačuvana u fajl mapa_{i+1}.pkl.")

generisi_mape()
