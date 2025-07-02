import core
import simulacija
import pickle

def ucitaj_mapu(fajl):
    with open(fajl, "rb") as f:
        mapa = pickle.load(f)
    return mapa

putanja, istorija_int_mapa = simulacija.simulacija(core.robot, core.robot["pozicija"], core.cilj, core.mapa)