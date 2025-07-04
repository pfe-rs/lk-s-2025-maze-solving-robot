import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt
import pygame
#import maze 
import pickle
import random
 

SCREEN_WIDTH, SCREEN_HEIGHT = 120, 60

#senzor_radius = 10
#cilj = np.array([15, 70])

def ucitaj_mapu(fajl):
    with open(fajl, "rb") as f:
        mapa = pickle.load(f)
    return mapa
 
def dvosobni_stan():
    
    mapa = {
        "celije": np.ones((60, 120)),
        "linije": []
    }
    
    mapa["celije"][10:40, 10:40] = 0
    mapa["celije"][10:40, 50:80] = 0
    mapa["celije"][20:30, 40:50] = 0

    start_point = np.array([15, 15])
    end_point = np.array([15, 70])

    return {
        'mapa': mapa,
        'start': start_point,
        'end': end_point
    }



def sacuvaj_mapu(fajl, mapa):
    with open("mapa.pkl", "wb") as f: 
        pickle.dump(mapa, f) # pretvara mapu u niz bajtova i zapisuje u fajl f 
        # cuvanje mape u pickle file 

def robot(pozicija):
    return {
        "pozicija": pozicija,
        "ugao": 0,
        "robot_brzina": 1
    }

def grid_mapa(celije, cilj):
    return {
        "celije": celije,
        "linije": None,
        "cilj": cilj
    }

def laser_mapa(linije, cilj):
    return {
        "celije": None,
        "linije": linije,
        "cilj": cilj 
    }

def senzor():
    return {
        "radius": 100, # koliko daleko moze laser da vidi.
        "num_lasers": 300 # broj lasera
    }


putanja = []
istorija_int_mapa = []

#putanja, istorija_int_mapa = simulacija(robot, robot["pozicija"], cilj, mapa)

#putanja.append(robot["pozicija"])

#vizuelizacija(mapa, putanja, istorija_int_mapa)

# DONE : razdvojiti simulaciju od vizuelizacije
# DONE : napraviti funkciju za simulaciju (pocetni polozaj robota, cilj, mapa) -> putanja, listu internih mapa
# DONE : napraviti funkciju za vizuelizaciju (mapa, putanja, lista internih mapa)
 