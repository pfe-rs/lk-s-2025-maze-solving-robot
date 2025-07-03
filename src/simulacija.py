import detekcija
import kretanje
import mapiranje
import core
import pickle
import numpy as np

from core import senzor


def simulacija(robot: dict, senzor: dict, mapa: dict):
    istorija_int_mapa = []
    putanja = []

    interna_mapa = core.grid_mapa(np.zeros(mapa["celije"].shape), mapa["cilj"])
    interni_robot = core.robot(np.array([0, 0]))

    for i in range(50):
        sken_data = detekcija.detekcija_matrix(senzor, robot, mapa)
        interna_mapa = mapiranje.mapiranje_matrix(interna_mapa, sken_data, robot)
        istorija_int_mapa.append(interna_mapa["celije"])
        robot, interni_robot = kretanje.kretanje(interni_robot, robot, mapa["cilj"], interna_mapa)
        print(i, robot["pozicija"])
        putanja.append(robot["pozicija"])


    return putanja, istorija_int_mapa

if __name__ == "__main__":


    map_data = core.ucitaj_mapu("dvosobni_stan.pkl")
    mapa = map_data["mapa"]
    robot = core.robot(map_data["start"])
    senzor = core.senzor()
    mapa["cilj"] = map_data["end"]

    putanja, istorija_int_mapa = simulacija(robot, senzor, mapa)

    with open("simulacija_rezultati.pkl", "wb") as f:
        pickle.dump({"putanja": putanja, "istorija_int_mapa": istorija_int_mapa}, f)
