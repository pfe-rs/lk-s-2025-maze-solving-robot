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
    return mapa

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

def grid_mapa(celije):
    return {
        "celije": celije,
        "linije": None,
    }

def laser_mapa(linije):
    return {
        "celije": None,
        "linije": linije,
    }



putanja = []
istorija_int_mapa = []

#putanja, istorija_int_mapa = simulacija(robot, robot["pozicija"], cilj, mapa)

#putanja.append(robot["pozicija"])

#vizuelizacija(mapa, putanja, istorija_int_mapa)

# DONE : razdvojiti simulaciju od vizuelizacije
# DONE : napraviti funkciju za simulaciju (pocetni polozaj robota, cilj, mapa) -> putanja, listu internih mapa
# DONE : napraviti funkciju za vizuelizaciju (mapa, putanja, lista internih mapa)

# TODO : dataset(sacuvane u fajlu) generisanih mapa, sa pocetnom i krajnjom tackom {pickle}
# TODO : interna reprezentacija robota, sken treba da je matrica dimenzija [2*sken radius][2*sken radius], promeniti mapiranje da radi sa tom matricom skena, 
# TODO : detekciju izmeniti, gledati da sken ne bude cela matrica vec samo deo koji je skeniran tj dimenzija [2*sken radius][2*sken radius], traziti odakle dokle ide for petlja itd


# TODO : detekcija => koristim pravu poziciju robota, sken se ogranicava na 20*20 umesto na celu mapu
# TODO : mapiranje => prosledjujem pravog robota, siftujem sken za +- radijus skena 