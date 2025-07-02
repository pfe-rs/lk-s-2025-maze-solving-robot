import detekcija
import kretanje
import mapiranje
from core import *
import pickle

with open("mapa.pkl", "wb") as f: 
    pickle.dump(mapa, f) #pretvara mapu u niz bajtova i zapisuje u fajl f 
    #cuvanje mape u pickle file 
print("mapa.pkl")

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

    with open("simulacija_rezultati.pkl", "wb") as f:
        pickle.dump({"putanja": putanja, "istorija_int_mapa": istorija_int_mapa}, f)


    return putanja, istorija_int_mapa