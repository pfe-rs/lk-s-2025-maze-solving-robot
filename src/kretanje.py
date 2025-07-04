import numpy as np
import core
import heapq


def bfs(interni_robot: dict, robot: dict, cilj: np.ndarray, interna_mapa: dict) -> tuple[float, float]:
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
                    if robot["pozicija"][0]==neighbor[0] and\
                        robot["pozicija"][1]==neighbor[1]: break
        else: continue
        break
    
    novi_cilj = visited[int(robot["pozicija"][0]), int(robot["pozicija"][1])]
    razlika = novi_cilj - robot["pozicija"]
    razlika = razlika / np.linalg.norm(razlika)
    pomeraj = razlika * robot["robot_brzina"]

    nova_poz_pravog_robota = robot["pozicija"] + pomeraj
    nova_poz_internog_robota = interni_robot["pozicija"] + pomeraj
    
    robot["pozicija"] = nova_poz_pravog_robota
    interni_robot["pozicija"] = nova_poz_internog_robota
    
    return robot, interni_robot
    

def dfs(interni_robot: dict, robot: dict, cilj: np.ndarray, interna_mapa: dict) -> tuple[float, float]:
    queue = [cilj.astype(int)]
    visited = np.zeros((interna_mapa["celije"].shape[0], interna_mapa["celije"].shape[1], 2))
    while len(queue) > 0:
        current_pos = queue.pop()
        for dir in np.array([(1, 0), (-1, 0), (0, 1), (0, -1)]):
            neighbor = current_pos + dir
            if 0 <= neighbor[0] < interna_mapa["celije"].shape[0] and 0 <= neighbor[1] < interna_mapa["celije"].shape[1]:
                if interna_mapa["celije"][neighbor[0]][neighbor[1]] == 0 and np.linalg.norm(visited[neighbor[0]][neighbor[1]]) == 0:
                    queue = queue + [neighbor]
                    visited[neighbor[0], neighbor[1]] = current_pos
                    if robot["pozicija"][0]==neighbor[0] and\
                        robot["pozicija"][1]==neighbor[1]: break
        else: continue
        break
    
    novi_cilj = visited[int(robot["pozicija"][0]), int(robot["pozicija"][1])]
    razlika = novi_cilj - robot["pozicija"]
    razlika = razlika / np.linalg.norm(razlika)
    pomeraj = razlika * robot["robot_brzina"]

    nova_poz_pravog_robota = robot["pozicija"] + pomeraj
    nova_poz_internog_robota = interni_robot["pozicija"] + pomeraj
    
    robot["pozicija"] = nova_poz_pravog_robota
    interni_robot["pozicija"] = nova_poz_internog_robota
    
    return robot, interni_robot


def heuristic(a, b):
    return np.linalg.norm(a - b)

def astar(internal_robot: dict, robot: dict, goal: np.ndarray, map_data: dict) -> tuple[float, float]:
    start = robot["pozicija"].astype(int)
    goal = goal.astype(int)
    
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start, None))
    
    visited = np.zeros((*map_data["celije"].shape, 2))
    cost_so_far = np.full(map_data["celije"].shape, np.inf)
    cost_so_far[start[0], start[1]] = 0

    while open_set:
        _, current_cost, current, parent = heapq.heappop(open_set)
        current = np.array(current)
        if parent is not None: parent = np.array(parent)

        if parent is not None:
            visited[current[0], current[1]] = parent

        if np.array_equal(current, goal):
            break

        for direction in [(1,0), (-1,0), (0,1), (0,-1)]:
            neighbor = current + direction
            x, y = neighbor
            if 0 <= x < map_data["celije"].shape[0] and 0 <= y < map_data["celije"].shape[1]:
                if map_data["celije"][x][y] == 0:
                    new_cost = current_cost + 1
                    if new_cost < cost_so_far[x, y]:
                        cost_so_far[x, y] = new_cost
                        priority = new_cost + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, new_cost, tuple(neighbor), tuple(current)))

    current_pos = robot["pozicija"].astype(int)
    parent = visited[current_pos[0], current_pos[1]]
    direction = parent - robot["pozicija"]
    direction = direction / np.linalg.norm(direction)
    step = direction * robot["robot_brzina"]
    
    print(step, end = "       ")

    robot["pozicija"] += step
    internal_robot["pozicija"] += step

    return robot, internal_robot

    
def kretanje(interni_robot: dict, robot: dict, cilj: np.ndarray, interna_mapa: dict) -> tuple[float, float]:
    return bfs(interni_robot, robot, cilj, interna_mapa)

