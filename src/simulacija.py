import detekcija
from kretanje import bfs, dfs, astar
from kretanje import dfs
import mapiranje
import core
import pickle
import numpy as np

from core import senzor


def simulacija(robot: dict, senzor: dict, mapa: dict):
    print(mapa["cilj"])
    
    istorija_int_mapa = []
    putanja = []

    interna_mapa = core.grid_mapa(np.zeros(mapa["celije"].shape), mapa["cilj"])
    interni_robot = core.robot(np.array([0, 0]))

    # TODO: Da petlja traje duze
    for i in range(300):
<<<<<<< HEAD
            putanja = []
            istorija_int_mapa = []
            if np.linalg.norm(robot["pozicija"] - mapa["cilj"]) < 1:
                print(f"Cilj dostignut u {i} koraka!")
                break 
            
            sken_data = detekcija.formiranjeSkena(senzor, robot, mapa)
            interna_mapa = mapiranje.mapiranje_matrix(interna_mapa, sken_data, robot)
            istorija_int_mapa.append(interna_mapa["celije"])
            
            robot, interni_robot = kretanje.kretanje(interni_robot, robot, mapa["cilj"], interna_mapa)

            print(f"Korak {i}: {robot['pozicija']}")
            putanja.append(robot["pozicija"]) 
=======
        sken_data = detekcija.formiranjeSkena(senzor, robot, mapa)
        interna_mapa = mapiranje.mapiranje_matrix(interna_mapa, sken_data, robot)
        istorija_int_mapa.append(interna_mapa["celije"])
        robot, interni_robot = astar(interni_robot, robot, mapa["cilj"], interna_mapa)
        # TODO: Da se prekine simulacija ako je cilj < 1 udaljen
        print(i, robot["pozicija"])
        putanja.append(robot["pozicija"])

        if abs(robot["pozicija"][0] - mapa["cilj"][0])<1 and abs(robot["pozicija"][1] - mapa["cilj"][1])<1: 
            break

>>>>>>> 0c5d1e9bc223dbd5c63b3295de4ba3228614c712

    return putanja, istorija_int_mapa


if __name__ == "__main__":

    # TODO: stavi for petlju da pokrene simulaciju za sve mape

<<<<<<< HEAD
    for i in range (1, 10):
        
=======
    for i in range(10):
>>>>>>> 0c5d1e9bc223dbd5c63b3295de4ba3228614c712
        map_data = core.ucitaj_mapu(f"mapa{i+1}.pkl")
        mapa = map_data["mapa"]
        robot = core.robot(map_data["start"])
        senzor = core.senzor()
        mapa["cilj"] = map_data["end"]

        putanja, istorija_int_mapa = simulacija(robot, senzor, mapa)

        print("Duzina:", len(putanja))

        with open(f"simulacija_rezultati{i+1}.pkl", "wb") as f:
            pickle.dump({"putanja": putanja, "istorija_int_mapa": istorija_int_mapa}, f)
