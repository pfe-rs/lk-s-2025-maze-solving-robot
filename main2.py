import pygame as py
import math
import random
from environment import buildMapu
from sensors import LaserSensor
from features import featuresDetection
import time

py.init()
screen = py.display.set_mode((1200, 600))
py.display.set_caption("SLAM Simulation")

def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))

FeatureMAP = featuresDetection()
environment = buildMapu((600, 1200))
laser = LaserSensor(800, environment.map.copy(), uncertanity=(0.5, 0.01))

environment.map.fill((0,0,0))  # crna pozadina

environment.infomap = environment.map.copy()
originalMap = environment.map.copy()

running = True
while running:
    time.sleep(0.1)
    environment.infomap = originalMap.copy()
    FeatureMAP.draw_line_features(environment.infomap)
    ENDPOINTS = [0, 0]
    sensorON = py.mouse.get_focused()
    PREDICTED_POINTS_TODRAW = []

    for event in py.event.get():
        if event.type == py.QUIT:
            running = False

    if sensorON:
        print("senzor je ON")
        position = py.mouse.get_pos()
        laser.position = position
        sensor_data = laser.sense_obstacles()
        FeatureMAP.laser_points_set(sensor_data)

        BREAK_POINT_IND = 0  # resetuj svaki put kad miš aktivira senzor

        while BREAK_POINT_IND < (FeatureMAP.NP - FeatureMAP.PMIN): # break point ind je indeks u listi laserpoints od kog pocinjem traziti novi seed segment, ne krecem ispocetka vec gledam nove podatke svaki put
            seedSeg = FeatureMAP.seed_segment_detection(laser.position, BREAK_POINT_IND) 
            if not seedSeg:
                break
            seedSegment, PREDICTED_POINTS_TODRAW, INDICES = seedSeg # indices je tuple koji kaze koje tacke su u pocetnom seed segmentu npr indices(12, 18) koristi tacke od 12 do 18
            results = FeatureMAP.seed_segment_growing(INDICES, BREAK_POINT_IND)
            if not results:
                BREAK_POINT_IND = INDICES[1]
                continue
            line_seg, line_eq, OUTERMOST, BREAK_POINT_IND, _, (m, c) = results
            ENDPOINTS[0] = FeatureMAP.projection_point2line(OUTERMOST[0], m, c)
            ENDPOINTS[1] = FeatureMAP.projection_point2line(OUTERMOST[1], m, c)

            COLOR = random_color()
            for point in line_seg:
                x, y = int(point[0][0]), int(point[0][1])
                if 0 <= x < 1200 and 0 <= y < 600:
                    py.draw.circle(environment.infomap, COLOR, (x, y), 2)
            py.draw.line(environment.infomap, (255, 0, 0), ENDPOINTS[0], ENDPOINTS[1], 2)
            environment.dataStorage(sensor_data)

    environment.map.blit(environment.infomap, (0, 0))
    screen.blit(environment.map, (0, 0))
    py.display.update()
 