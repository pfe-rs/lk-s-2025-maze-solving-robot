import numpy as np
ROBOT_SPEED=1
def kretanje(robot_pozicija: tuple[float, float], cilj: np.ndarray, interna_mapa: np.ndarray) -> tuple[float, float]:
    queue = [cilj.astype(int)]
    visited = np.zeros((interna_mapa.shape[0], interna_mapa.shape[1], 2))
    while len(queue) > 0:
        current_pos = queue.pop()
        for dir in np.array([(1, 0), (-1, 0), (0, 1), (0, -1)]):
            neighbor = current_pos + dir
            if 0 <= neighbor[0] < interna_mapa.shape[0] and 0 <= neighbor[1] < interna_mapa.shape[1]:
                if interna_mapa[neighbor[0]][neighbor[1]] == 0 and np.linalg.norm(visited[neighbor[0]][neighbor[1]]) == 0:
                    queue = [neighbor] + queue
                    visited[neighbor[0], neighbor[1]] = current_pos
                    if neighbor[0] == robot_pozicija[0] and neighbor[1] == robot_pozicija[1]: break
        else: continue
        break


    novi_cilj = visited[int(robot_pozicija[0]), int(robot_pozicija[1])]
    razlika = novi_cilj - robot_pozicija
    razlika = razlika / np.linalg.norm(razlika)
    pomeraj = razlika * ROBOT_SPEED
    return pomeraj.astype(int)