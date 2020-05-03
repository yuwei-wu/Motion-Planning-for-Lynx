import logging

import os
from os import path

from random import random, randint
from math import sqrt, pow, atan2, sin, cos

import cv2  # pip install opencv-python


logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(message)s')
log = logging.getLogger('rrt')

dir_path = os.path.dirname(os.path.realpath(__file__))


class Graph:
    def __init__(self):
        self.vertexes = set()
        self.edges = []
        self.inverse_tracking = {}

    def init(self, q_init):
        self.vertexes.add(q_init)

    def add_vertex(self, q):
        self.vertexes.add(q)

    def add_edge(self, qx, qy):
        self.edges.append((qx, qy))

        if qy in self.inverse_tracking:
            log.info('!!! conflict found %s -> %s', qx, qy)
        self.inverse_tracking[qy] = qx


class RRTTemplate:
    """ https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
    """
    def __init__(self, start, goal, Pr=0.6, K=1000, Δq=1):
        self.start = start
        self.goal = goal
        self.graph = Graph()

        self.Pr = Pr
        self.K = K
        self.Δq = Δq  # incremental distance

    def random_conf(self):
        """ grabs a random configuration q-rand while rejecting those in space
            using some collision detection algorithm.
        """
        pass

    def nearest_vertex(self, q_rand, graph):
        """ runs through all vertices v in graph G, calculates the distance between
            q-rand and v using some measurement function thereby returning the nearest vertex
        """
        return min(
            (self.distance(q, q_rand), q) for q in graph.vertexes
        )[1]

    def new_conf(self, q_near, q_rand):
        """ selects a new configuration q-new by moving an incremental distance Δq
         from q-near in the direction of q-rand.
        """
        d_x = q_rand[0] - q_near[0]
        d_y = q_rand[1] - q_near[1]

        θ = atan2(d_x, d_y)
        q_new = (q_near[0] + round(sin(θ) * self.Δq),
                 q_near[1] + round(cos(θ) * self.Δq))

        # check if the path through q_near to q_rand obstruct free
        r = 0
        while r <= self.Δq:
            pt = (q_near[0] + round(sin(θ) * r), q_near[1] + round(cos(θ) * r))
            if not self.obstruct_free(pt):
                return
            r += 0.5  # step
        return q_new

    def distance(self, qx, qy):
        return sqrt(pow(qx[0] - qy[0], 2) + pow(qx[1] - qy[1], 2))

    def obstruct_free(self, qx):
        pass

    def build(self):
        self.graph.init(self.start)

        for k in range(self.K):
            p = random()  # (0, 1]
            if p < self.Pr:
                q_rand = self.goal
            else:
                q_rand = self.random_conf()
            # log.debug('%s iteration: get rand q %s', k, q_rand)

            q_near = self.nearest_vertex(q_rand, self.graph)
            # log.debug('found nearest q %s', q_near)

            q_new = self.new_conf(q_near, q_rand)
            if not (q_new and self.obstruct_free(q_new)):
                continue

            if q_new in self.graph.vertexes:
                log.info("ignore duplicate q %s", q_new)
                continue

            log.debug('get a new q %s', q_new)
            self.graph.add_vertex(q_new)
            self.graph.add_edge(q_near, q_new)

            # We have reached the goal if the distance less than a given threshold
            if self.distance(q_new, self.goal) < self.Δq:
                log.info('Finally we reach the goal!')
                if q_new != self.goal:
                    self.graph.add_vertex(self.goal)
                    self.graph.add_edge(q_new, self.goal)
                return

    def find_path(self):
        if self.goal not in self.graph.inverse_tracking:
            return

        path = []
        cur = self.goal
        # XXX: There is a bug here, we would enter dead loop if there is a cycle in path
        while True:
            path.insert(0, cur)
            prev = self.graph.inverse_tracking.get(cur)
            if not prev:
                break
            cur = prev
        return path


####
# Used to testing text mode maze
####

class Map(object):
    """ A simple N * N map
    """
    def __init__(self):
        self.pixel_map = []
        self.n = 0

    def load(self, content):
        print('load map...')
        for line in content.split('\n'):
            if not line.strip(): continue
            row = [e for e in line.strip().split(' ')]
            self.pixel_map.append(row)
        # black magic
        self.n = len(row)

    def display(self):
        print('\n'.join(' '.join(e) for e in self.pixel_map))


Obstruction = '+'
Path_Mark = '@'


TEXT_MAP = '''
+ + + + + + + + + + + + + + + + + + + + + + + + + + + + + +
+ . . . . . . . . . . . . . . . . . . . . . . . . . . . . +
+ . . . . . . . . . . . . . . . . . . . . . . . . . . . . +
+ . . . . . . . . . . . . . . . . . . . . . . . . . . . . +
+ + + + . . . . + . . . . . . . . . . . . . . . . . . . . +
+ . . . . . . . + . . . . . . . . . . . . . . . . . . . . +
+ . . . . . . . + . . . . . . . . . . . . . . . . . . . . +
+ . . . . . . + + + + + + + + . . . . . . . . . . . . . . +
+ . . . . . . . + . . . . . . . . . . . . . . . . . . . . +
+ . . . . . . . + . . . . . . . . . . . . . . . . . . . . +
+ . . . . . . . + . . . . . . . . . . . . . . . . . . . . +
+ + + . . . . . + . . . . . . . . . . . . . . . . . . . . +
+ . . . . . . . + . . . . + + + + + + . . . . . . . . . . +
+ . . . . . . . + + + + . + . . . . + + . . . . . . . + . +
+ . . . . . . . . . . + + + . . . . . . . . . . . . . + . +
+ . . . . . . . . . . . . . . . . . . . . . . . . . . + . +
+ . . . . . . . . . . . . . . . . . . . . + + + + + + + + +
+ . . . . . . . . . . . . . . . . . . . . . . . . . . . . +
+ . . . . . . . . . . . + + + . . . . . . . . . . . . . . +
+ . . . . . . . . . . . + . + . . . . . . . . . . . . . . +
+ . . . . . . . . . . . + . + . . . . . . . . . . . . . . +
+ . . . . . . . . . . . + . + . . . . . . . . . . . . . . +
+ . . . . . . . . . . . + . + . . . . . . + + + . . . . . +
+ . . . . . . . . . . . + . + . . . . . . + + + . . . . . +
+ . . . . . . . . . . . + . + . . . . . . + + + . . . . . +
+ . . . . . . . . . . . + . + . . . . . . + + + . . . . . +
+ . . . . . . . . . . . . . . . . . . . . + + + . . . . . +
+ . . . . . . . . . . . + . + . . . . . . . . . . . . . . +
+ . . . . . . . . . . . + . + . . . . . . . . . . . . . . +
+ + + + + + + + + + + + + + + + + + + + + + + + + + + + + +
'''


class TextRTT(RRTTemplate):
    def __init__(self, start, goal, map, **kwargs):
        super(TextRTT, self).__init__(start, goal, **kwargs)
        self.map = map

    def random_conf(self):
        return randint(1, self.map.n-1), randint(1, self.map.n-1)

    def obstruct_free(self, qx):
        return self.map.pixel_map[qx[0]][qx[1]] != Obstruction

    def display(self):
        path = self.find_path()
        if path:
            for qx in path:
                self.map.pixel_map[qx[0]][qx[1]] = 'X'
            self.map.display()


####
# Used to testing image mode
####

class CV2RTT(RRTTemplate):
    def __init__(self, start, goal, img_file, **kwargs):
        super(CV2RTT, self).__init__(start, goal, **kwargs)
        self.img_file = img_file
        self.img = cv2.imread(img_file, 0)  # ndarray
        log.info("load image file %s, shape %s", img_file, self.img.shape)

    def random_conf(self):
        return randint(1, self.img.shape[0]-1), randint(1, self.img.shape[1]-1)

    def obstruct_free(self, qx):
        # note that the position of x, y in ndarray is opposite
        ans = qx[0] < self.img.shape[0] and\
               qx[1] < self.img.shape[1] and\
               self.img[qx[1], qx[0]] == 255
        return ans

    def display(self):
        img = cv2.imread(self.img_file)
        path = self.find_path()
        if path:
            color = (15, 0, 255)
            for i in range(len(path)-1):
                cv2.line(img, path[i], path[i+1], color, 1)
                cv2.circle(img, path[i], 2, color, -1)
            cv2.circle(img, path[-1], 2, color, -1)
            cv2.imshow('Path Planing', img)
            cv2.waitKey(0)


def test_text_mode_maze():
    map = Map()
    map.load(TEXT_MAP)
    rtt = TextRTT((1, 1), (28, 28), map, Δq=1)
    rtt.build()
    rtt.display()


def test_cv2_map_2():
    # test map2
    rtt = CV2RTT((1, 1), (498, 498), path.join(dir_path, 'images/map2.bmp'), Δq=20, K=10000)
    rtt.build()
    rtt.display()


def test_cv2_map_3():
    # test map3
    rtt = CV2RTT((1, 1), (220, 429), path.join(dir_path, 'images/map3.bmp'), Δq=20, K=10000)
    rtt.build()
    rtt.display()


if __name__ == '__main__':
    test_cv2_map_3()
