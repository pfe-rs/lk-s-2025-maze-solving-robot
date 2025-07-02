import detekcija
import kretanje
import mapiranje
import core
import pickle
import numpy as np

from core import senzor_radius


def simulacija(robot: dict, cilj: np.ndarray, mapa: dict):
    istorija_int_mapa = []
    putanja = []

    interna_mapa = core.grid_mapa(np.zeros(mapa["celije"].shape))
    interni_robot = core.robot(np.array([0, 0]))

    for i in range(50):
        sken = detekcija.detekcija(senzor_radius, robot, mapa)
        interna_mapa["celije"] = mapiranje.mapiranje(interna_mapa, sken, robot)
        istorija_int_mapa.append(interna_mapa["celije"])
        robot["pozicija"], interni_robot["pozicija"] = kretanje.kretanje(interni_robot, robot, cilj, interna_mapa)
        print(i, robot["pozicija"])
        putanja.append(robot["pozicija"])


    return putanja, istorija_int_mapa

if __name__ == "__main__":

    mapa = {
        "celije": np.ones((60, 120)),
        "linije": []
    }
    
    mapa["celije"][10:40, 10:40] = 0
    mapa["celije"][10:40, 50:80] = 0
    mapa["celije"][20:30, 40:50] = 0

    map_data = core.ucitaj_mapu("mapa.pkl")
    mapa = map_data["mapa"]
    robot = core.robot(map_data["start"])
    cilj = map_data["end"]

    putanja, istorija_int_mapa = simulacija(robot, cilj, mapa)

    with open("simulacija_rezultati.pkl", "wb") as f:
        pickle.dump({"putanja": putanja, "istorija_int_mapa": istorija_int_mapa}, f)
