import pygame as py
import math
import numpy as np
import maze


def uncertanity_add(distance, angle, sigma):
    mean = np.array([distance, angle])
    covariace = np.diag(sigma ** 2)
    distance, angle = np.random.multivariate_normal(mean, covariace)
    distance = max(distance, 0)
    angle = max(angle, 0)
    return [distance, angle]

class LaserSensor:
    def __init__(self, range, map, uncertanity):
        self.Range = range
        self.speed = 4
        self.sigma = np.array([uncertanity[0], uncertanity[1]])
        self.position = (0, 0)
        self.map = map
        self.sirina, self.visina = py.display.get_surface().get_size()
        self.senseObstacles = []

    def EuclidianDistIzmedjuTacaka(self, obstaclePosition):
        dx = (obstaclePosition[0] - self.position[0]) ** 2
        dy = (obstaclePosition[1] - self.position[1]) ** 2
        return math.sqrt(dx + dy)

    def sense_obstacles(self):
        data = []
        x1, y1 = self.position[0], self.position[1]
        for angle in np.linspace(0, 2 * math.pi, 60):
            x2 = x1 + self.Range * math.cos(angle)
            y2 = y1 - self.Range * math.sin(angle)
            for i in range(0, 100):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.sirina and 0 < y < self.visina:
                    color = self.map.get_at((x, y))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        distance = self.EuclidianDistIzmedjuTacaka((x, y))
                        output = uncertanity_add(distance, angle, self.sigma)
                        output.append(self.position)
                        data.append(output)
                        break
        return data
