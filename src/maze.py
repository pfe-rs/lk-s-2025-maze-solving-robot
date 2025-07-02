import random
import numpy as np
import pygame

# Podesavanja
SCREEN_WIDTH, SCREEN_HEIGHT = 120, 60
ROOM_COUNT = 4
MIN_ROOM_SIZE, MAX_ROOM_SIZE = 50, 400

# Klasa za sobu
class Room:
    def __init__(self, rect):
        self.rect = rect
        self.center = (rect.x + rect.w // 2, rect.y + rect.h // 2)

    def collides(self, other):
        return self.rect.colliderect(other.rect)

# Generisanje mape
def generate_map():
    rooms = []
    tries = 0
    while len(rooms) < ROOM_COUNT and tries < 1000:
        w = random.randint(MIN_ROOM_SIZE, MAX_ROOM_SIZE)
        h = random.randint(MIN_ROOM_SIZE, MAX_ROOM_SIZE)
        x = random.randint(0, SCREEN_WIDTH - w)
        y = random.randint(0, SCREEN_HEIGHT - h)
        new_room = Room(pygame.Rect(x, y, w, h))

        if all(not new_room.collides(r) for r in rooms):
            rooms.append(new_room)
        tries += 1

    # Poveži sobe
    rooms.sort(key=lambda r: r.center[0])
    corridors = []
    for i in range(1, len(rooms)):
        x1, y1 = rooms[i-1].center
        x2, y2 = rooms[i].center
        if random.choice([True, False]):
            corridors.append(((x1, y1), (x2, y1)))
            corridors.append(((x2, y1), (x2, y2)))
        else:
            corridors.append(((x1, y1), (x1, y2)))
            corridors.append(((x1, y2), (x2, y2)))

    # Generisanje matrice
    matrix = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=int)

    # Obeleži sobe u matrici
    for room in rooms:
        for x in range(room.rect.left, room.rect.right):
            for y in range(room.rect.top, room.rect.bottom):
                matrix[y, x] = 1
    
    # Obeleži hodnike u matrici
    for (x1, y1), (x2, y2) in corridors:
        if x1 == x2:  # vertikalno
            for y in range(min(y1, y2), max(y1, y2) + 1):
                matrix[y, x1] = 2
        else:  # horizontalno
            for x in range(min(x1, x2), max(x1, x2) + 1):
                matrix[y1, x] = 2

    return matrix

# Generisanje i čuvanje matrica
def generate_and_save_maps(num_maps=5):
    for i in range(num_maps):
        # Generisanje matrice
        matrix = generate_map()

        # Čuvanje matrice
        matrix_filename = f"map_{i + 1}_matrix.txt"
        np.savetxt(matrix_filename, matrix, fmt="%d", delimiter=" ")
        print(f"Matrica za mapu {i + 1} je sačuvana kao {matrix_filename}.")

# Pokreni generisanje mapa
generate_and_save_maps()
