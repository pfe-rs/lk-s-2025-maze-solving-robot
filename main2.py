import pygame as py
import math
import random
import time

from environment import buildMapu
from sensors import LaserSensor
from features import featuresDetection

py.init()
screen = py.display.set_mode((1200, 600))
py.display.set_caption("SLAM Simulation")
clock = py.time.Clock()  # kontrolaaa frejmova u sekundi, otp 1 frame po Bozicu i Uskrsu

def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))

def procesiraj_lidar_podatke(laser, FeatureMAP, environment, sensor_data):
    BREAK_POINT_IND = 0
    ENDPOINTS = [0, 0]

    while BREAK_POINT_IND < (FeatureMAP.NP - FeatureMAP.PMIN):
        seedSeg = FeatureMAP.seed_segment_detection(laser.position, BREAK_POINT_IND)
        if not seedSeg:
            break

        seedSegment, predicted_points, INDICES = seedSeg
        results = FeatureMAP.seed_segment_growing(INDICES, BREAK_POINT_IND)
        if not results:
            BREAK_POINT_IND = INDICES[1]
            continue

        line_seg, line_eq, OUTERMOST, BREAK_POINT_IND, _, (m, c) = results
        ENDPOINTS[0] = FeatureMAP.projection_point2line(OUTERMOST[0], m, c)
        ENDPOINTS[1] = FeatureMAP.projection_point2line(OUTERMOST[1], m, c)

       # COLOR = random_color()
        #for point in line_seg:
         #   x, y = int(point[0][0]), int(point[0][1])
          #  if 0 <= x < 1200 and 0 <= y < 600:
           #     py.draw.circle(environment.infomap, COLOR, (x, y), 2)

       # py.draw.line(environment.infomap, (255, 0, 0), ENDPOINTS[0], ENDPOINTS[1], 2)
        environment.dataStorage(sensor_data)


FeatureMAP = featuresDetection()
environment = buildMapu((600, 1200))
laser = LaserSensor(800, environment.map.copy(), uncertanity=(0.0, 0.0))

environment.map.fill((0,0,0))  # crna pozadina
environment.infomap = environment.map.copy()

running = True
while running:
  #  screen.fill((0, 0, 0))
 #   environment.infomap = environment.map.copy()

    ENDPOINTS = [0, 0] #### krajnje tacke trenutne detektovane linije
    sensorON = py.mouse.get_focused()
    PREDICTED_POINTS_TODRAW = [] #### lista za crtanje predvidjenih tacaka

    for event in py.event.get():
        if event.type == py.QUIT:
            running = False
      #  elif event.type == py.KEYDOWN:
       #     if event.key == py.K_g:
        #        FeatureMAP.spoji_globalno_sve_linije()

    if sensorON:
        position = py.mouse.get_pos()
        laser.position = position
        sensor_data = laser.sense_obstacles()
        FeatureMAP.laser_points_set(sensor_data)

        procesiraj_lidar_podatke(laser, FeatureMAP, environment, sensor_data)

    FeatureMAP.nacrtaj_segmentirano_spojene_linije(environment.infomap)
    environment.map.blit(environment.infomap, (0, 0))
    screen.blit(environment.map, (0, 0))
    py.display.update()
    clock.tick(15)

