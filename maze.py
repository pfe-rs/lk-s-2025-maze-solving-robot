import pygame
import random
import sys

# Podesavanja svega
SCREEN_WIDTH, SCREEN_HEIGHT = 1200, 600
ROOM_COUNT = 25
MIN_ROOM_SIZE, MAX_ROOM_SIZE = 50, 400
ROOM_COLOR = (255, 255, 255)
CORRIDOR_COLOR = (255, 255, 255)  
BG_COLOR = (0, 0, 0)

pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
clock = pygame.time.Clock()

# Klasa za sobu
class Room:
    def __init__(self, rect):
        self.rect = rect
        self.center = (rect.x + rect.w // 2, rect.y + rect.h // 2)

    def collides(self, other):
        return self.rect.colliderect(other.rect)

# Pravimo sobe
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

# Crtanje
def draw():
    screen.fill(BG_COLOR)
    for room in rooms:
        pygame.draw.rect(screen, ROOM_COLOR, room.rect)
    for (x1, y1), (x2, y2) in corridors:
        pygame.draw.line(screen, CORRIDOR_COLOR, (x1, y1), (x2, y2), 40)

# Nacrtaj i sacuvaj sliku
draw()
pygame.display.flip()
pygame.image.save(screen, "lavirint.png")
print("Slika je sačuvana kao 'lavirint.png'.")

pygame.time.wait(5000)
pygame.quit()
