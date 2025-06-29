import math
import pygame as py
import random
import heapq
from environment import buildMapu
from sensors import LaserSensor

py.init()

environment = buildMapu((600, 1200))
environment.originalMap = environment.eksternaMapa.copy() #pravimo kopiju mape koju cemo da koristimo 
environment.infomap.fill((0, 0, 0)) #ovo je crna mapa koja ide preko ovih
laser = LaserSensor(200, environment.originalMap, uncertanity=(0.5, 0.01))

"""
Ovaj laser koji smo unjeli radi na originalnoj mali i ima greske
Ova prva stvar u zagradi kod uncertanity je standardna devijacija,
a ova druga je dodatna greska, nesto kao greska ugla... """

clock = py.time.Clock()

#DJELIMO MAPU NA GRID 

CELL_SIZE = 10
grid_width = environment.originalMap.get_width() // CELL_SIZE
grid_height = environment.originalMap.get_height() // CELL_SIZE

#DEFINISEMO DA LI JE CELIJA PROHODNA

def is_cell_walkable(cx, cy): # cx i cy su indeksi celija na koje smo podjelili mapu
    margin = 2 # imamo malo lufta oko celije
    start_x = max(cx * CELL_SIZE - margin, 0)
    end_x = min((cx + 1) * CELL_SIZE + margin, environment.originalMap.get_width()-1)
    start_y = max(cy * CELL_SIZE - margin, 0)
    end_y = min((cy + 1) * CELL_SIZE + margin, environment.originalMap.get_height()-1)

# ako u toj celiji ima crnih polja kaze da nije prohodna, tj. vraca false

    for x in range(start_x, end_x):
        for y in range(start_y, end_y):
            if environment.originalMap.get_at((x, y))[:3] == (0, 0, 0):
                return False
    return True

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

#racuna koliko ima po y osi da se krece i po x osi da se krece do cilja

def astar(start, goal):
    open_set = [] #ovde cemo da stavljamo ljubicaste tackice koje nadjemo
    heapq.heappush(open_set, (0, start))
    """
    A star koristi specifican algoritam heapq, koji kada unesemo ljubicaste tacke, on ih sortira po tome koja je najbliza
    0 znači "f-score" početne ćelije pri ubacivanju u open set — da bi start bio prvi izvučen za procesiranje.

    """
    came_from = {}
    g_score = {start: 0}
    #na pocetku nam je g score 0, jer se jos nismo kretali, a cijeli put koji treba da predjemo se nalazi u heuristic
    f_score = {start: heuristic(start, goal)}
    """
    A star radi tako sto gleda koliko je do sada presao i koliko mu ostaje da predje. To je f_score,
    on je koliko je robot presao od starta do nekog mjesta ka cilju (g_score) i koliko mu je ostalo da predje do cilja (heuristic)
    """
    while open_set:
        current = heapq.heappop(open_set)[1]
        # posto je na prvom mjestu open_seta najbliza ljubicasta celija, vadimo je

        if current == goal: #kada dodjemo do ljubicaste celije do koje smo htjeli da dodjemo
            path = [] #onda pravimo listu u koju cemo da ubacimo putanju kojom smo dosli od cilja do starta
            while current in came_from:
                path.append(current)
                current = came_from[current]
                """
                Pratimo sve tacke koje smo prosli unazad pomocu rjecnika came_from
                """
            path.append(start)
            path.reverse()
            return path
        """
        Dodajemo start na tu putanju i okrecemo je tako da imamo put od starta do cilja
        """

        x, y = current #uzimamo poziciju celije koju trenutno istrazujemo 
        neighbors = []
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < grid_width and 0 <= ny < grid_height:
                if is_cell_walkable(nx, ny):
                    neighbors.append((nx, ny))
                    #provjeravamo da li su susjedne celije progodne, tj. 8 susjednih celija u svim smjerovima, ako jesu dodajemo ih na listu 

        for neighbor in neighbors:
            tentative_g = g_score[current] + (1.4 if neighbor[0]!=x and neighbor[1]!=y else 1)
            #za celije koje nisu susjedne po x i y osi dodaje tg=1.4, kao dijagonala na kvadrat 1 sa 1, u suprotnom sabira 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                #ako ranije nismo posjetili put do susjednog polja, ili smo nasli kraci put do njega, azuriramo
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

#LASER

def draw_laser_beam(surface, start_pos, angle, max_distance, color):
    last_valid = None
    for i in range(0, max_distance):
        x = int(start_pos[0] + i * math.cos(angle))
        y = int(start_pos[1] - i * math.sin(angle))
        #racunamo zrake duz pravaca
        if not (0 <= x < surface.get_width() and 0 <= y < surface.get_height()):
            break
        #ako izadje van ekrana prekida
        map_color = environment.originalMap.get_at((x, y))[:3]
        info_color = environment.infomap.get_at((x, y))[:3]
        if map_color == (0, 0, 0) or info_color == (255, 0, 0):
            break
        #ako je udario u crnu ili crvenu laser staje
        if info_color == (0, 0, 0):
            environment.infomap.set_at((x, y), color)
            #ako je na info mapi udario od crno mjesto, boji ga u zuto
        last_valid = (x, y)
    if last_valid: #provjerava da li postoji neka tacka na koju je laser naisao a da nije udario u zid
        final_color = environment.originalMap.get_at(last_valid)[:3] #boja na pravoj, originalnoj mapi na toj tacki
        current_info = environment.infomap.get_at(last_valid)[:3] #boja na mapi koju je laser nacrtao 
        if final_color == (0, 0, 0) and current_info == color:
            environment.infomap.set_at(last_valid, (255, 0, 0))
#ako je na mapi sa laserom zuta, a na originalnoj mapi crna, onda tu stavlja crvenu tacku, jer je to zid

def ljubicasta(surface, center, radius=75): #definisemo ljubicaste tackice
    width, height = surface.get_width(), surface.get_height()
    cx, cy = center
    xmin = max(1, cx - radius)
    xmax = min(width - 1, cx + radius)
    ymin = max(1, cy - radius)
    ymax = min(height - 1, cy + radius)
    surface.lock() #zakljucavamo sliku za brzi prilaz pikselima
    to_remove = [] #ljubicasti pikseli koji vise nisu granica
    to_add = [] #novi ljubicasti pikseli 
    for x in range(xmin, xmax):
        for y in range(ymin, ymax):
            current_color = surface.get_at((x, y))[:3] #gledamo boju piksela na toj poziciji
            if current_color == (200, 0, 200):
                has_yellow = False
                has_black = False
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        #gledamo susjedne piksele
                        if dx == 0 and dy == 0:
                            continue
                        #dok smo u istom redu ili koloni kao ranije:
                        nx, ny = x + dx, y + dy #trenutne koordinate za koliko su se pomjerili
                        if 0 <= nx < width and 0 <= ny < height:
                            c = surface.get_at((nx, ny))[:3] #kupi boju pozadine
                            if c == (255, 255, 0): 
                                has_yellow = True
                            if c == (0, 0, 0):
                                has_black = True
                if not (has_yellow and has_black):
                    to_remove.append((x, y))
                    #ukoliko vise nema crne i zute komsije, to znaci da ta ljubicasta tacka vise nije granicna i ona se skida
            elif current_color == (255, 255, 0): #ako je neka tacka zuta, gledamo piksele oko nje
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            c = surface.get_at((nx, ny))[:3]
                            if c == (0, 0, 0):
                                to_add.append((nx, ny))
                                #ako je susjedna celika do zute celije crna, onda tu dodajemo ljubicastu tacku
    for pos in to_remove:
        surface.set_at(pos, (255, 255, 0))
    for pos in to_add:
        if surface.get_at(pos)[:3] not in [(255, 255, 0), (255, 0, 0)]:
            surface.set_at(pos, (200, 0, 200))
    surface.unlock()
    #ako pozicija izmedju crvene i crne nije zuta (istrazeno) ili crvena-zid, on ce da je pretvori u ljubicasto 

def krug(surface, pos, radius):
    x0, y0 = int(pos[0]), int(pos[1]) #centar kruga zaokruzen na cijele brojeve
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1): 
            x, y = x0 + dx, y0 + dy
            if 0 <= x < surface.get_width() and 0 <= y < surface.get_height():
                if math.hypot(dx, dy) <= radius: #provjerava da li je tacka unutar kruga, ne samo u kvadratu
                    if surface.get_at((x, y))[:3] == (0, 0, 0):
                        return False
                    #ako naidje na crnu, tu je prepreka, ako ne naidje, vraca true
    return True

class Robot:
    def __init__(self, x, y, laser, mapa, infomap):
        self.x = x
        self.y = y
        self.angle = random.uniform(0, 2 * math.pi) #gleda u nasumicnom pravcu kada se stvori
        self.vel = 3 #brzina
        self.rot_step = math.radians(50)
        self.laser = laser #povezuje laser sa robotom
        self.map = mapa
        self.infomap = infomap #pamti obe mape
        self.clearance = 10 #minimalna udaljenost koju robot treba da ima od prepreka
        self.path = [] #trenutna putanja koju robot treba da prati 
        self.astar_cooldown = 10 
        self.last_positions = [] #pamti posljednjih nekoliko pozicija da vidi da li se zaglavio, qko se ne pobjera dovoljno dugo

    def get_pos(self):
        return int(self.x), int(self.y)
    #trenutna pozicija robota

    def detect_collision(self, pos):
        return not krug(self.map, pos, self.clearance)
    #Ako se robot slobodno krece false, ako se udario true

    def move_forward(self):
        new_x = self.x + self.vel * math.cos(self.angle)
        new_y = self.y - self.vel * math.sin(self.angle)
        if not self.detect_collision((new_x, new_y)):
            self.x, self.y = new_x, new_y
            #ako nema sudara, robot se pomjera
        else:
            found_new_angle = False #ako ima sudara, robot ne moze naprijed pa trazi novi ugao
            for direction in [1, -1]: #okrece se udesno (1) ili ulijevo (-1)
                for i in range(1, 6): 
                    new_angle = self.angle + direction * math.radians(i * 15)
                    #provjerava ostale uglove
                    test_x = self.x + self.vel * math.cos(new_angle)
                    test_y = self.y - self.vel * math.sin(new_angle)
                    #novi pravac
                    if not self.detect_collision((test_x, test_y)):
                        self.angle = new_angle
                        self.x, self.y = test_x, test_y
                        found_new_angle = True
                        break
                    #testira da li je okej da ide tim novim pravcem
                if found_new_angle:
                    break
            if not found_new_angle:
                self.angle += random.choice([-1, 1]) * self.rot_step

    def update(self):
        self.laser.position = self.get_pos() #postavljamo poziciju lasera na trenutnu poziciju robota
        data = self.laser.sense_obstacles() #laser osluskuje okolinu i vraca podatke o udaljenosti do prepreka
        if data:
            environment.dataStorage(data) #ako je detektovao zid, to se cuva za kasniju obradu
        for a in range(0, 360, 20):
            angle_rad = math.radians(a)
            draw_laser_beam(self.infomap, self.get_pos(), angle_rad, 200, (255, 255, 0))
            #crta laser 

    def get_visible_purple_front(self, radius=200, fov_deg=360):
        visible = [] #ovde cemo ubacivati vidljive ljubicaste tacke
        cx, cy = self.get_pos() # pozicija robota
        for dx in range(-radius, radius):
            for dy in range(-radius, radius):
                x = cx + dx
                y = cy + dy
                #pretvara relativne koordinate u apsolutne 
                if 0 <= x < self.infomap.get_width() and 0 <= y < self.infomap.get_height():
                    if self.infomap.get_at((x, y))[:3] == (200, 0, 200):
                        angle_to = math.atan2(y - cy, x - cx)
                        angle_diff = (angle_to - self.angle + math.pi * 3) % (2 * math.pi) - math.pi
                        if abs(angle_diff) <= math.radians(fov_deg) / 2:
                            if self.has_line_of_sight((cx, cy), (x, y)):
                                visible.append((x, y))
                                #pronalazi ljubicaste piksele
        return visible

    def has_line_of_sight(self, start, end):
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while (x, y) != (x1, y1):
            if self.map.get_at((x, y))[:3] == (0, 0, 0):
                return False
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True

    def move_toward(self, target):
        tx, ty = target
        dx = tx - self.x
        dy = self.y - ty
        angle_to_target = math.atan2(dy, dx)
        self.angle = angle_to_target
        new_x = self.x + self.vel * math.cos(self.angle)
        new_y = self.y - self.vel * math.sin(self.angle)
        if not self.detect_collision((new_x, new_y)):
            self.x, self.y = new_x, new_y
        else:
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
        if self.astar_cooldown > 0:
            self.astar_cooldown -= 1

        if not self.path:
            if self.astar_cooldown == 0:
                purple_points = self.get_visible_purple_front(radius=150, fov_deg=360)
                if purple_points:
                    nearest = min(purple_points, key=lambda p: math.hypot(self.x - p[0], self.y - p[1]))
                    start = (int(self.x // CELL_SIZE), int(self.y // CELL_SIZE))
                    goal = (nearest[0] // CELL_SIZE, nearest[1] // CELL_SIZE)
                    path = astar(start, goal)
                    if path and len(path) > 1:
                        self.path = path[1:12]  # više koraka unaprijed
                        self.astar_cooldown = 40
                    else:
                        self.move_forward()
                else:
                    self.move_forward()
            else:
                self.move_forward()
        else:
            next_cell = self.path[0]
            target_x = next_cell[0] * CELL_SIZE + CELL_SIZE // 2
            target_y = next_cell[1] * CELL_SIZE + CELL_SIZE // 2

            if not is_cell_walkable(next_cell[0], next_cell[1]):
                self.path = []
                self.move_forward()
                return

            self.move_toward((target_x, target_y))
            dist = math.hypot(self.x - target_x, self.y - target_y)
            if dist < 5:
                self.path.pop(0)

            self.last_positions.append((self.x, self.y))
            if len(self.last_positions) > 10:
                self.last_positions.pop(0)
                dx = max(pos[0] for pos in self.last_positions) - min(pos[0] for pos in self.last_positions)
                dy = max(pos[1] for pos in self.last_positions) - min(pos[1] for pos in self.last_positions)
                if dx < 3 and dy < 3:
                    self.path = []
                    self.angle += random.choice([-1, 1]) * self.rot_step

            new_x = self.x + self.vel * math.cos(self.angle)
            new_y = self.y - self.vel * math.sin(self.angle)
            if self.detect_collision((new_x, new_y)):
                self.path = []

robot = None
running = True
frame_count = 0

while running:
    frame_count += 10
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
        if frame_count % 30 == 0:
            ljubicasta(environment.infomap, robot.get_pos(), radius=75)

    environment.map.blit(environment.infomap, (0, 0))
    if robot:
        py.draw.circle(environment.map, (255, 105, 180), robot.get_pos(), 5)
    py.display.update()

    clock.tick(30)
 