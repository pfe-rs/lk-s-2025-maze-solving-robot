import pygame as py
import math
import random
import heapq
from environment import buildMapu  
from sensors import LaserSensor
from features import featuresDetection
import time    

py.init()  

environment = buildMapu((600, 1200))  
environment.originalMap = environment.eksternaMapa.copy()
environment.infomap.fill((0, 0, 0))
laser = LaserSensor(200, environment.originalMap, uncertanity=(0.5, 0.01))
FeatureMAP = featuresDetection()  

# prvi broj u uncertanity zagradi definiše standardnu devijaciju, a drugi dodatni faktor nesigurnosti

clock = py.time.Clock() 

CELL_SIZE = 10  # dimenzija celije mreze u pikselima
grid_width = environment.originalMap.get_width() // CELL_SIZE  # sirina mreze u celijama
grid_height = environment.originalMap.get_height() // CELL_SIZE  # visina mreze u celijama

def is_cell_walkable(cx, cy):
    # Provjerava da li je ćelija u mreži "prohodna" tj. bez prepreka
    margin = 2  # zadebljanje oko zida, da ne bi udarao u zid
    
    start_x = max(cx * CELL_SIZE - margin, 0)
    end_x = min((cx + 1) * CELL_SIZE + margin, environment.originalMap.get_width()-1)
    start_y = max(cy * CELL_SIZE - margin, 0)
    end_y = min((cy + 1) * CELL_SIZE + margin, environment.originalMap.get_height()-1)

    # Prolazi kroz sve piksele unutar te oblasti
    for x in range(start_x, end_x):
        for y in range(start_y, end_y):
            if environment.originalMap.get_at((x, y))[:3] == (0, 0, 0): # gleda di je prepreka
                return False
    return True  # Ako nije pronađena prepreka, ćelija je prohodna

def heuristic(a, b):
    # Heuristicka funkcija za A* algoritam, koristi Manhattansku distancu
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(start, goal):
    # implementacija a star za pronalazak najkraceg puta od start do krajnje celije

    open_set = []  # red sa cvorovima koje treba straziti
    heapq.heappush(open_set, (0, start))  # dodaje start ćeliju sa f_score = 0

    came_from = {}  # Mapa koja pamti prethodni čvor za svaki posećeni čvor (za rekonstrukciju puta)
    g_score = {start: 0}  # Cijena prelaska od starta do date ćelije (g)
    f_score = {start: heuristic(start, goal)}  # Procena ukupne cijene do cilja (f = g + h)

    while open_set:
        current = heapq.heappop(open_set)[1]  # Uzima ćeliju sa najmanjim f_score

        if current == goal:  
            # Ako je stigao do cilja, rekonstruise putanju
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()  # putanja od starta do cilja
            return path

        x, y = current
        neighbors = []  # Lista susednih celija koje su prohodne

        # Provjerava susjedne ćelije u 8 pravaca (4 ortogonalne + 4 dijagonalne)
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            nx, ny = x+dx, y+dy
            # Provera da li je susjedna ćelija u okviru mreže
            if 0 <= nx < grid_width and 0 <= ny < grid_height:
                if is_cell_walkable(nx, ny):  # Provjerava da li je ćelija prohodna
                    neighbors.append((nx, ny))

        for neighbor in neighbors:
            # Trošak prelaska do susjeda je 1 za ortogonalne, 1.4 za dijagonalne (približno korjenu iz 2)
            tentative_g = g_score[current] + (1.4 if neighbor[0]!=x and neighbor[1]!=y else 1)
            # Ako je nova cijena prelaska manja od prethodne, ili susjed nije posjećen
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current  # Pamti prethodni čvor
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)  # Procena cijene do cilja
                heapq.heappush(open_set, (f_score[neighbor], neighbor))  # Dodaje susjeda u open set
    return None  # Ako nema putanje

def draw_laser_beam(surface, start_pos, angle, max_distance, color):
    # Crta laserski zrak na datoj površini od start_pos pod uglom 'angle' do max_distance
    last_valid = None
    for i in range(0, max_distance):
        # Izračunava poziciju sledećeg piksela na liniji zraka
        x = int(start_pos[0] + i * math.cos(angle))
        y = int(start_pos[1] - i * math.sin(angle))  

        # Provjera da li je pixel u granicama mape
        if not (0 <= x < surface.get_width() and 0 <= y < surface.get_height()):
            break

        map_color = environment.originalMap.get_at((x, y))[:3]
        info_color = environment.infomap.get_at((x, y))[:3]

        # Ako je prepreka ili info boja crvena, prekida se crtanje zraka
        if map_color == (0, 0, 0) or info_color == (255, 0, 0):
            break

        # Ako je trenutna boja info mape crna (prazna), postavlja se boja lasera
        if info_color == (0, 0, 0):
            environment.infomap.set_at((x, y), color)

        last_valid = (x, y)  # Pamti poslednju validnu poziciju zraka

    if last_valid:
        final_color = environment.originalMap.get_at(last_valid)[:3]
        current_info = environment.infomap.get_at(last_valid)[:3]

        # Ako je na poslednjem pikselu prepreka i boja info mape je ista kao boja zraka,
        # on će da mjenja boju u ljubičastu (200,0,200) i to je kao neka korica oko žutog sloja
        if final_color == (0, 0, 0) and current_info == color:
            environment.infomap.set_at(last_valid, (200, 0, 200))

def update_purple_edges(surface, center, radius=75):
    # Osvježava ivice ljubičastih područja u blizini centra u datom radijusu
    width, height = surface.get_width(), surface.get_height()
    cx, cy = center

    # def podrucje koje updatuje sa tim tackama
    xmin = max(1, cx - radius)
    xmax = min(width - 1, cx + radius)
    ymin = max(1, cy - radius)
    ymax = min(height - 1, cy + radius)

    surface.lock()  

    to_remove = []  # Liste za pozicije koje treba ukloniti ili dodati
    to_add = []

    for x in range(xmin, xmax):
        for y in range(ymin, ymax):
            current_color = surface.get_at((x, y))[:3]

            if current_color == (200, 0, 200):  # Ako je ljubičasta tačka
                has_yellow = False
                has_black = False
                # Proverava okolne piksele 3x3 oko te tačke
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            c = surface.get_at((nx, ny))[:3]
                            if c == (255, 255, 0):
                                has_yellow = True
                            if c == (0, 0, 0):
                                has_black = True
                # Ako nema susednog žutog i crnog, tačka se uklanja
                if not (has_yellow and has_black):
                    to_remove.append((x, y))

            elif current_color == (255, 255, 0):  # Ako je žuta tačka
                # Dodaje u ljubičaste sve susjedne crne tačke, u smislu crne tacke koje predstavljaju neistrazena polja u infomapi
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            c = surface.get_at((nx, ny))[:3]
                            if c == (0, 0, 0):
                                to_add.append((nx, ny))

    for pos in to_remove:
        surface.set_at(pos, (0, 0, 0))  # Uklanja ljubičaste koje nisu potrebne

    for pos in to_add:
        # Dodaje ljubičastu tamo gde nije već žuta ili crvena
        if surface.get_at(pos)[:3] not in [(255, 255, 0), (255, 0, 0)]:
            surface.set_at(pos, (200, 0, 200))

    surface.unlock()  # Otključava površinu

def is_clear_circle(surface, pos, radius):
    # Proverava da li je krug oko pozicije 'pos' sa zadatim radijusom slobodan od crnih piksela (prepreka)
    x0, y0 = int(pos[0]), int(pos[1])
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            x, y = x0 + dx, y0 + dy
            if 0 <= x < surface.get_width() and 0 <= y < surface.get_height():
                if math.hypot(dx, dy) <= radius:  # Ako je piksel unutar kruga
                    if surface.get_at((x, y))[:3] == (0, 0, 0):  # Ako je prepreka
                        return False
    return True  # Ako nema preprekq, prostor je čist

class Robot:
    def __init__(self, x, y, laser, mapa, infomap):
        # Konstruktor robota - postavlja početne koordinate, ugao i druge parametre
        self.x = x
        self.y = y
        self.angle = random.uniform(0, 2 * math.pi)  # Nasumični početni ugao
        self.vel = 6  # Brzina kretanja 
        self.rot_step = math.radians(5)  # Korak rotacije (u radijanima)
        self.laser = laser  # Referenca na laserski senzor
        self.map = mapa  # Referenca na mapu prepreka
        self.infomap = infomap  # Referenca na info mapu za crtanje i detekciju
        self.clearance = 5  # Minimalni razmak za detekciju prepreka oko robota
        self.path = []  # Lista koraka do cilja koju će da koristi A* algoritmom
        self.astar_cooldown = 0  # Brojač za čekanje između A* pretraga 
        self.last_positions = []  # Pamćenje poslednjih pozicija za detekciju zaglavljenosti

    def get_pos(self):
        # Vraća trenutnu poziciju robota kao cjelobrojne koordinate
        return int(self.x), int(self.y)

    def detect_collision(self, pos):
        # Provjerava da li je pozicija 'pos' koliziona sa preprekama
        return not is_clear_circle(self.map, pos, self.clearance)

    def move_forward(self):
        # Pokušava da pomjeri robota unaprijed u pravcu trenutnog ugla
        new_x = self.x + self.vel * math.cos(self.angle)
        new_y = self.y - self.vel * math.sin(self.angle)

        if not self.detect_collision((new_x, new_y)):
            # Ako nema prepreka, pomjeri se
            self.x, self.y = new_x, new_y
        else:
            # Ako postoji prepreka, pokusava da pronadje alternativni ugao pomjeranja
            found_new_angle = False
            for direction in [1, -1]:  # Pokusava u obe strane (lijevo i desno)
                for i in range(1, 7):  # Pokusava uglove od 15°, 30°, ..., 90°
                    new_angle = self.angle + direction * math.radians(i * 15)
                    test_x = self.x + self.vel * math.cos(new_angle)
                    test_y = self.y - self.vel * math.sin(new_angle)
                    if not self.detect_collision((test_x, test_y)):
                        # Ako nađe slobodan put, postavlja ugao i pomjera se
                        self.angle = new_angle
                        self.x, self.y = test_x, test_y
                        found_new_angle = True
                        break
                if found_new_angle:
                    break
            # Ako nije našao nikakav slobodan ugao, rotira se nasumično za rot_step
            if not found_new_angle:
                self.angle += random.choice([-1, 1]) * self.rot_step

    
    def update(self):
        self.laser.position = self.get_pos()
        data = self.laser.sense_obstacles()
        if data:
            FeatureMAP.laser_points_set(data)

            BREAK_POINT_IND = 0
            while BREAK_POINT_IND < (FeatureMAP.NP - FeatureMAP.PMIN):
                seedSeg = FeatureMAP.seed_segment_detection(self.get_pos(), BREAK_POINT_IND)
                if not seedSeg:
                    break
                seedSegment, predicted_points, INDICES = seedSeg
                results = FeatureMAP.seed_segment_growing(INDICES, BREAK_POINT_IND)
                if not results:
                    BREAK_POINT_IND = INDICES[1]
                    continue
                line_seg, line_eq, OUTERMOST, BREAK_POINT_IND, _, (m, c) = results
                ENDPOINTS = [
                    FeatureMAP.projection_point2line(OUTERMOST[0], m, c),
                    FeatureMAP.projection_point2line(OUTERMOST[1], m, c),
                ]
                COLOR = (255, 0, 0)
                for point in line_seg:
                    x, y = int(point[0][0]), int(point[0][1])
                    if 0 <= x < environment.infomap.get_width() and 0 <= y < environment.infomap.get_height():
                        py.draw.circle(self.infomap, COLOR, (x, y), 2)
                py.draw.line(self.infomap, (255, 0, 0), ENDPOINTS[0], ENDPOINTS[1], 2)
            FeatureMAP.reset_points()
            
    def get_visible_purple_front(self, radius=200, fov_deg=360):
        # Vraća sve ljubičaste tačke unutar radijusa i polja vida
        visible = []
        cx, cy = self.get_pos()

        for dx in range(-radius, radius):
            for dy in range(-radius, radius):
                x = cx + dx
                y = cy + dy
                if 0 <= x < self.infomap.get_width() and 0 <= y < self.infomap.get_height():
                    if self.infomap.get_at((x, y))[:3] == (200, 0, 200):  # Ljubičasta tačka
                        angle_to = math.atan2(y - cy, x - cx)  # Ugao do tačke
                        # Izračunava razliku ugla između ugla robota i tačke, normalizovano na [-pi, pi]
                        angle_diff = (angle_to - self.angle + math.pi * 3) % (2 * math.pi) - math.pi
                        if abs(angle_diff) <= math.radians(fov_deg) / 2:  # Provjera da li je u FOV
                            if self.has_line_of_sight((cx, cy), (x, y)):  # Provjera linije vida
                                visible.append((x, y))
        return visible

    def has_line_of_sight(self, start, end):
        # Provjerava da li postoji neometana linija vida između start i end tačke
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while (x, y) != (x1, y1):
            if self.map.get_at((x, y))[:3] == (0, 0, 0):  # Ako postoji prepreka na liniji
                return False
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True  # Linija vida je čista

    def move_toward(self, target):
        # Pomera robota ka ciljnoj poziciji 'target'
        tx, ty = target
        dx = tx - self.x
        dy = self.y - ty  # y se smanjuje zbog koord. sistema
        angle_to_target = math.atan2(dy, dx)  # Izračunava ugao ka cilju
        self.angle = angle_to_target

        new_x = self.x + self.vel * math.cos(self.angle)
        new_y = self.y - self.vel * math.sin(self.angle)

        if not self.detect_collision((new_x, new_y)):
            self.x, self.y = new_x, new_y  # Pomjera se ako nema prepreka
        else:
            # Ako postoji prepreka, traži alternativni ugao kao u move_forward()
            found_new_angle = False
            for direction in [1, -1]:
                for i in range(1, 7):
                    new_angle = self.angle + direction * math.radians(i * 15)
                    test_x = self.x + self.vel * math.cos(new_angle)
                    test_y = self.y - self.vel * math.sin(new_angle)
                    if not self.detect_collision((test_x, test_y)):
                        self.angle = new_angle
                        self.x, self.y = test_x, test_y
                        found_new_angle = True
                        break
                if found_new_angle:
                    break
            if not found_new_angle:
                self.angle += random.choice([-1, 1]) * self.rot_step

    def update_movement(self):
        # Glavna logika kretanja robota, koristi A* za navigaciju ka ljubičastim tačkama

        if self.astar_cooldown > 0:
            self.astar_cooldown -= 2  # Smanjuje cooldown između poziva A*

        if not self.path:  # Ako nema definisan put
            if self.astar_cooldown == 0:
                purple_points = self.get_visible_purple_front(radius=200, fov_deg=360)  # Pronalazi ljubičaste tačke u vidokrugu
                if purple_points:
                    # Pronalazi najbližu ljubičastu tačku
                    nearest = min(purple_points, key=lambda p: math.hypot(self.x - p[0], self.y - p[1]))
                    # Pravi start i goal ćelije za A*
                    start = (int(self.x // CELL_SIZE), int(self.y // CELL_SIZE))
                    goal = (nearest[0] // CELL_SIZE, nearest[1] // CELL_SIZE)
                    path = astar(start, goal)  # Izračunava put
                    if path and len(path) > 1:
                        self.path = path[1:6]  # Uzima prvih max 5 koraka
                        self.astar_cooldown = 12  # Postavlja cooldown da ne računa prečesto
                    else:
                        self.move_forward()  # Ako nema putanje, kreće se napred
                else:
                    self.move_forward()  # Ako nema ljubičastih tačaka, ide napred
            else:
                self.move_forward()  # Ako je cooldown, ide napred
        else:
            next_cell = self.path[0]  # Uzima sledeću ćeliju na putu
            target_x = next_cell[0] * CELL_SIZE + CELL_SIZE // 2  # Centar ciljne ćelije X
            target_y = next_cell[1] * CELL_SIZE + CELL_SIZE // 2  # Centar ciljne ćelije Y

            if not is_cell_walkable(next_cell[0], next_cell[1]):
                # Ako ćelija nije hodljiva, robot briše putanju i ide napred
                self.path = []
                self.move_forward()
                return

            self.move_toward((target_x, target_y))  # Pomera se ka sledećoj ćeliji

            dist = math.hypot(self.x - target_x, self.y - target_y)  # Proverava udaljenost od cilja
            if dist < 5:
                self.path.pop(0)  # Ako je blizu, uklanja tu ćeliju iz puta

            # Pamti poslednjih 20 pozicija da detektuje da li je zaglavljen (malo pomjeranja)
            self.last_positions.append((self.x, self.y))
            if len(self.last_positions) > 20:
                self.last_positions.pop(0)
                dx = max(pos[0] for pos in self.last_positions) - min(pos[0] for pos in self.last_positions)
                dy = max(pos[1] for pos in self.last_positions) - min(pos[1] for pos in self.last_positions)
                if dx < 3 and dy < 3:  # Ako se skoro uopšte nije pomjerio
                    self.path = []  # Resetuje putanju
                    self.angle += random.choice([-1, 1]) * self.rot_step  # robot se rotira nasumično

            # ako se sudari u pokretu, resetuje putanju
            new_x = self.x + self.vel * math.cos(self.angle)
            new_y = self.y - self.vel * math.sin(self.angle)
            if self.detect_collision((new_x, new_y)):
                self.path = []

robot = None  
running = True
frame_count = 0

while running:
    frame_count += 1

    for event in py.event.get():
        if event.type == py.QUIT:
            running = False

        elif event.type == py.MOUSEBUTTONDOWN:
            pos = py.mouse.get_pos()
            color = environment.originalMap.get_at(pos)[:3]

            if color == (255, 255, 255):  
                laser.position = pos
                robot = Robot(pos[0], pos[1], laser, environment.originalMap, environment.infomap)
                print("Robot postavljen na:", pos)

    if robot:
        robot.update()
        robot.update_movement()
        environment.show_sensorData()

        if frame_count % 5 == 0:
            # svakih 5 frejmova ažurira ljubičaste ivice oko robota u radijusu 75 piksela
            update_purple_edges(environment.infomap, robot.get_pos(), radius=75)

    environment.map.blit(environment.infomap, (0, 0))

    if robot:
        py.draw.circle(environment.map, (255, 205, 190), robot.get_pos(), 5)

    py.display.update()

    clock.tick(100)