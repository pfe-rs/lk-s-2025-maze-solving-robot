import math
import pygame as py
import maze

class buildMapu:
    def __init__(self, dimenzijeMape):
        py.init()
        self.listaLidartacaka = []
        self.eksternaMapa = py.image.load('lavirint.png')
        self.mapvisina, self.mapsirina = dimenzijeMape
        self.imeProzoraMape = 'Lidar simulacija'
        py.display.set_caption(self.imeProzoraMape)
        self.map = py.display.set_mode((self.mapsirina, self.mapvisina))
        self.map.blit(self.eksternaMapa, (0, 0))
        self.infomap = self.map.copy()
        self.originalMap = self.eksternaMapa.copy() 

        self.crna = (0, 0, 0)
        self.bela = (255, 255, 255)
        self.roze = (255, 192, 203)
        self.crvena = (255, 0, 0)

    def is_walkable(self, pos): #### dodala sam funkciju koja proverava prohodnost celije jer why not
        x, y = pos
        if 0 <= x < self.originalMap.get_width() and 0 <= y < self.originalMap.get_height():
            return self.originalMap.get_at((x, y))[:3] == (255, 255, 255)
        return False

    def RawAngleDisData2Coordinates(self, distance, angle, robotPosition):
        x = distance * math.cos(angle) + robotPosition[0]
        y = -distance * math.sin(angle) + robotPosition[1]
        return (int(x), int(y))

    def dataStorage(self, data):
        for element in data:
            point = self.RawAngleDisData2Coordinates(element[0], element[1], element[2])
            if point not in self.listaLidartacaka:
                self.listaLidartacaka.append(point)

    def show_sensorData(self):
        for point in self.listaLidartacaka:
            x, y = int(point[0]), int(point[1])
            width, height = self.infomap.get_size()
            if 0 <= x < width and 0 <= y < height:
                self.infomap.set_at((x, y), (255, 192, 203)) 
