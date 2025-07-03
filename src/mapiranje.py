from core import *


def mapiranje_matrix(interna_mapa: dict, sken_data: dict, robot: dict) -> dict:
    sken_matrix = sken_data["sken_matrix"]
    nova_celija = 1 - (1 - interna_mapa["celije"]) * (1 - sken_matrix)
    interna_mapa["celije"] = nova_celija

    return interna_mapa


def mapiranje_lines(interna_mapa: dict, sken_data: dict, robot: dict) -> dict:

    linije = []  
    for hit in sken_data["sken_hit"]:
        linije.append(hit) 

    interna_mapa["linije"] = linije
    return interna_mapa 
