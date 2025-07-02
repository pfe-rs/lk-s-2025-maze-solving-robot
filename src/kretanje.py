import numpy as np
import core

def kretanje(interni_robot: dict, robot: dict, cilj: np.ndarray, interna_mapa: dict) -> tuple[float, float]:
    queue = [cilj.astype(int)]
    visited = np.zeros((interna_mapa["celije"].shape[0], interna_mapa["celije"].shape[1], 2))
    while len(queue) > 0:
        current_pos = queue.pop()
        for dir in np.array([(1, 0), (-1, 0), (0, 1), (0, -1)]):
            neighbor = current_pos + dir
            if 0 <= neighbor[0] < interna_mapa["celije"].shape[0] and 0 <= neighbor[1] < interna_mapa["celije"].shape[1]:
                if interna_mapa["celije"][neighbor[0]][neighbor[1]] == 0 and np.linalg.norm(visited[neighbor[0]][neighbor[1]]) == 0:
                    queue = [neighbor] + queue
                    visited[neighbor[0], neighbor[1]] = current_pos


    novi_cilj = visited[int(robot["pozicija"][0]), int(robot["pozicija"][1])]
    razlika = novi_cilj - robot["pozicija"]
    razlika = razlika / np.linalg.norm(razlika)
    pomeraj = razlika * robot["robot_brzina"]
    return robot["pozicija"] + pomeraj, interni_robot["pozicija"] + pomeraj
