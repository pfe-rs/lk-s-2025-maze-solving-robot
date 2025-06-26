import numpy as np
import math
from fractions import Fraction
from scipy.odr import *
import pygame as py
   
class featuresDetection:
    def __init__(self):
        self.EPSILON = 4 # max odstupanje od linije
        self.DELTA = 4 # granica koliko neka Lidar tacka dobro lezi ili ne lezi na liniji  
        self.SNUM = 5  # broj pocetnih tacaka u seed segmentu
        self.PMIN = 12 # min broj tacaka da bi segment bio bolidan
        self.LMIN = 10 # min duzina linijskog segmenta
        self.GMAX = 10 # max rastojanje medju susednim tackama
        self.LMAX = 200
        self.GUSTOCA_PRAGA = 0.05

        self.SEED_SEGMENTS = []
        self.LINE_SEGMENTS = []
        self.LASERPOINTS = []
        self.LINE_PARAMS = None
        self.NP = len(self.LASERPOINTS) - 1 # broj tacaka
        self.LR = 0 # duzina trenutnog segmenta
        self.PR = 0 # broj tacaka u trenutnom segmentu
        self.LINE_FEATURES = []

    def euklidijanovaDistancaizmedjuPoints(self, point1, point2): # samo distanca dve tacke koje se nalaze na razlicitim koordinatama u coord sistemu
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def distancaPointaodLinije(self, params, point):
        A, B, C = params
        return abs(A * point[0] + B * point[1] + C) / math.sqrt(A ** 2 + B ** 2)

    def dist_point2point(self, point1, point2): # isto sto i funkcija euklidijanovaDist... samo drugacije ime
        return self.euklidijanovaDistancaizmedjuPoints(point1, point2)

    def line2points(self, m, b):
        x1 = 5
        y1 = m * x1 + b
        x2 = 2000
        y2 = m * x2 + b
        return [(x1, y1), (x2, y2)]

    def lineFormGen2SlopeIntercept(self, A, B, C): # generalna forma lnije sluzi za potrebna geometrijska racunanja, a slope intercept nam je porebna jer je bolja za predstavljanje tj iscrtavanje linija zbog nagiba
        m = -A / B
        b = -C / B
        return m, b

    def lineFormSi2Gen(self, m, b):
        A, B, C = -m, 1, -b
        if A < 0:
            A, B, C = -A, -B, -C

        den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]
        Gcd = np.gcd(den_a, den_c)
        lcm = den_c * den_a / Gcd

        A *= lcm
        B *= lcm
        C *= lcm

        return A, B, C

    def lineIntersectGenForm(self, params1, params2):
        a1, b1, c1 = params1
        a2, b2, c2 = params2
        x = (c1 * b2 - b1 * c2) / (b1 * a2 - a1 * b2)
        y = (a1 * c2 - a2 * c1) / (b1 * a2 - b2 * a1)
        return x, y

    def points2line(self, point1, point2): ## od dve tacke pravi liniju oblika y = mx+b
        if point2[0] == point1[0]:
            return None, None
        m = (point2[1] - point1[1]) / (point2[0] - point1[0])
        b = point2[1] - m * point2[0]
        return m, b

    def orthogonal_projection_point2line(self, point, m, b): ## racuna gde bi tacka pala na liniju kad bi pala oid pravim uglom, koristi se da racuna gde bi tacka trebalo da bude da je idealno na liniji
        x, y = point
        m2 = -1 / m
        c2 = y - m2 * x
        intersection_x = -(b - c2) / (m - m2)
        intersection_y = m2 * intersection_x + c2
        return intersection_x, intersection_y

    def projection_point2line(self, point, m, b):
        return self.orthogonal_projection_point2line(point, m, b) ## isto kao prosla funkcija samo eto lakse mi ovako

    def RawAngleDisData2Coordinates(self, distance, angle, robotPosition):
        x = distance * math.cos(angle) + robotPosition[0]
        y = -distance * math.sin(angle) + robotPosition[1]
        return (int(x), int(y))

    def laser_points_set(self, data): ## pretvara sirove lidar podatke u tacke na mapi
        self.LASERPOINTS = []
        if data:
            for point in data:
                coordinates = self.RawAngleDisData2Coordinates(point[0], point[1], point[2])
                self.LASERPOINTS.append([coordinates, point[1]])
        self.NP = len(self.LASERPOINTS) - 1

    def linear_func(self, p, x):
        m, b = p
        return m * x + b

    def odr_fit(self, laser_points): ## radi regresiju tj crtanje linije kroz tacke
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points])
        linear_model = Model(self.linear_func)
        data = RealData(x, y)
        odr_model = ODR(data, linear_model, beta0=[0., 0.])
        out = odr_model.run()
        m, b = out.beta
        return m, b

    def predictPoint(self, line_params, sensed_point, robotpos): # funkcija prvo pravi liniju izmedju pozicije robota i tacke tj lidar zrak a potom racuna presek tog zraka i fitting linije, taj presek he predvidjena pozicija gde bi tacka trebala da bude ako je sve savrseno. zatim se radi poredjenje kako bismo videli odstupanje
        m, b = self.points2line(robotpos, sensed_point)
        if m is None:
            return sensed_point
        params1 = self.lineFormSi2Gen(m, b)
        predx, predy = self.lineIntersectGenForm(params1, line_params) # line_params je fitting linija zida dobijena iz odr fitovanja
        return predx, predy # gde se sece lidar znak i fitting linija. zelimo da vidimo da li se stavrna ldiar tacka poklapa sa tackom preseka, tj da li ldiar vidi zid bas tamo gde fitting linija to ocekuje. ako su predaleko ta tacka onda ne pripada liniji i koristimo delta da to odlucimo

    def seed_segment_detection(self, robot_position, break_point_ind):
        self.NP = max(0, self.NP)
        self.SEED_SEGMENTS = []
        for i in range(break_point_ind, (self.NP - self.PMIN)):
            flag = True
            predicted_points_to_draw = []
            j = i + self.SNUM
            m, c = self.odr_fit(self.LASERPOINTS[i:j])
            params = self.lineFormSi2Gen(m, c)

            for k in range(i, j):
                predicted_point = self.predictPoint(params, self.LASERPOINTS[k][0], robot_position)
                predicted_points_to_draw.append(predicted_point)

                d1 = self.distancaPointaodLinije(params, predicted_point)
                if d1 > self.DELTA:
                    flag = False
                    break

                d2 = self.distancaPointaodLinije(params, predicted_point)
                if d2 > self.EPSILON:
                    flag = False
                    break

            if flag:
                self.LINE_PARAMS = params
                return [self.LASERPOINTS[i:j], predicted_points_to_draw, (i, j)]

        return False

    def seed_segment_growing(self, indices, break_point):
        line_eq = self.LINE_PARAMS
        i, j = indices
        PB, PF = max(break_point, i - 1), min(j + 1, len(self.LASERPOINTS) - 1)

        while self.distancaPointaodLinije(line_eq, self.LASERPOINTS[PF][0]) < self.EPSILON:
            if PF > self.NP - 1:
                break
            m, b = self.odr_fit(self.LASERPOINTS[PB:PF])
            line_eq = self.lineFormSi2Gen(m, b)
            POINT = self.LASERPOINTS[PF][0]
            PF += 1
            NEXTPOINT = self.LASERPOINTS[PF][0]
            if self.euklidijanovaDistancaizmedjuPoints(POINT, NEXTPOINT) > self.GMAX:
                break

        PF -= 1

        while self.distancaPointaodLinije(line_eq, self.LASERPOINTS[PB][0]) < self.EPSILON:
            if PB < break_point:
                break
            m, b = self.odr_fit(self.LASERPOINTS[PB:PF])
            line_eq = self.lineFormSi2Gen(m, b)
            POINT = self.LASERPOINTS[PB][0]
            PB -= 1
            NEXTPOINT = self.LASERPOINTS[PB][0]
            if self.dist_point2point(POINT, NEXTPOINT) > self.GMAX:
                break

        PB += 1

        LR = self.dist_point2point(self.LASERPOINTS[PB][0], self.LASERPOINTS[PF][0])
        PR = len(self.LASERPOINTS[PB:PF])
        gustoća = PR / (LR + 1e-5)

        if (LR >= self.LMIN) and (PR >= self.PMIN) and (LR <= self.LMAX) and (gustoća >= self.GUSTOCA_PRAGA):
            self.LINE_PARAMS = line_eq
            m, b = self.lineFormGen2SlopeIntercept(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line2points(m, b)
            self.LINE_SEGMENTS.append((self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]))

            start_point = self.LASERPOINTS[PB + 1][0]
            end_point = self.LASERPOINTS[PF - 1][0]

            self.LINE_FEATURES.append({
                'start': start_point,
                'end': end_point,
                'params_gen': line_eq, # parametri linije u general form kao ax+by+c=0
                'params_si': (m, b), # parametri linije u slope intercept form gde je m nagib linije, a b mesto je gde linija presrece tj interceptuje y (predikcija y za dato x) 
                'points': self.LASERPOINTS[PB:PF] # sve tacke koje pripadaju segmentu linije od pb do pf
            })

            return [self.LASERPOINTS[PB:PF], self.two_points, (start_point, end_point), PF, line_eq, (m, b)]
        else:
            return False

    def draw_line_features(self, surface):
        # Crtanje svih sirovih LIDAR tačaka (cyan)
        for point in self.LASERPOINTS:
            py.draw.circle(surface, (0, 255, 255), point[0], 1)

        # Crtanje detektovanih linija (crveno)
        for feature in self.LINE_FEATURES:
            start = feature['start']
            end = feature['end']
            py.draw.line(surface, (255, 0, 0), start, end, 2)