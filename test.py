import pygame as py
import random
import math
import numpy as np
from environment import buildMapu
from sensors import LaserSensor

py.init()
screen = py.display.set_mode((1200, 600))
py.display.set_caption("Test Lidar")

environment = buildMapu((600, 1200))
environment.originalMap = environment.eksternaMapa.copy()
environment.map.fill((0, 0, 0))

laser = LaserSensor(300, environment.originalMap, uncertanity=(0.5, 0.01))

random_points = []
while len(random_points) < 5:
    x = random.randint(0, environment.originalMap.get_width() - 1)
    y = random.randint(0, environment.originalMap.get_height() - 1)
    if environment.is_walkable((x, y)):
        random_points.append((x, y))

for pt in random_points:
    laser.position = pt
    data = laser.sense_obstacles()
    for d, a, origin in data:
        x, y = environment.RawAngleDisData2Coordinates(d, a, origin)
        if 0 <= x < environment.infomap.get_width() and 0 <= y < environment.infomap.get_height():
            py.draw.circle(environment.infomap, (255, 0, 0), (x, y), 2)


for pt in random_points:
    py.draw.circle(environment.infomap, (0, 0, 255), pt, 5)

environment.map.blit(environment.infomap, (0, 0))
screen.blit(environment.map, (0, 0))
py.display.update()

running = True
while running:
    for event in py.event.get():
        if event.type == py.QUIT:
            running = False
