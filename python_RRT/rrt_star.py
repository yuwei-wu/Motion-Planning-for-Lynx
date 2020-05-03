import logging

from random import random
from math import sqrt, pow, atan2, sin, cos


log = logging.getLogger('rrt')


class RRTree:
    def __init__(self):
        # vertex -> cost from root node
        self.vertexes = {}  # used to check if new node is already pre-existing in the tree
        self.path = {}  # a list of <point, parent point> pair

    def init(self, q_init):
        self.add_node(q_init, None, 0)  # -1 means no parent

    def add_node(self, q, q_parent, cost):
        self.vertexes[q] = cost
        self.path[q] = q_parent


class RRTTemplate:
    """ https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
    """
    def __init__(self, start, goal, Pr=0.6, K=1000, Δq=1):
        self.start = start
        self.goal = goal
        self.rrtree = RRTree()

        self.Pr = Pr
        self.K = K
        self.Δq = Δq  # incremental distance

    def random_conf(self):
        """ grabs a random configuration q-rand while rejecting those in space
            using some collision detection algorithm.
        """
        pass

    def nearest_vertex(self, q_rand):
        """ runs through all vertices v in graph G, calculates the distance between
            q-rand and v using some measurement function thereby returning the nearest vertex

            Returns:
                 the q-nearest
        """
        return min(
            (self.distance_cost(q, q_rand), q)
            for q in self.rrtree.vertexes
        )[1]

    def new_conf(self, q_near, q_rand):
        """ selects a new configuration q-new by moving an incremental distance Δq
         from q-near in the direction of q-rand.
        """
        d_x = q_rand[0] - q_near[0]
        d_y = q_rand[1] - q_near[1]

        # direction to extend sample to produce new node
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

    def path_obstruct_free(self, qx, qy):
        d_x = qx[0] - qy[0]
        d_y = qx[1] - qy[1]

        # direction to extend sample to produce new node
        θ = atan2(d_x, d_y)

        # check if the path through q_near to q_rand obstruct free
        r = 0
        while r <= self.Δq:
            pt = (qx[0] + round(sin(θ) * r), qx[1] + round(cos(θ) * r))
            if not self.obstruct_free(pt):
                return False
            r += 0.5  # step
        return True

    def distance_cost(self, qx, qy):
        return sqrt(pow(qx[0] - qy[0], 2) + pow(qx[1] - qy[1], 2))

    def obstruct_free(self, qx):
        pass

    def build(self):
        self.rrtree.init(self.start)

        for k in range(self.K):
            p = random()  # (0, 1]
            if p < self.Pr:
                q_rand = self.goal
            else:
                q_rand = self.random_conf()
            # log.debug('%s iteration: get rand q %s', k, q_rand)

            q_near = self.nearest_vertex(q_rand)
            # log.debug('found nearest q %s', q_near)

            q_new = self.new_conf(q_near, q_rand)
            if not (q_new and self.obstruct_free(q_new)):
                continue

            # XXX: avoid cycle
            if q_new in self.rrtree.vertexes:
                log.debug("ignore pre-existing node %s in rrtree path", q_new)
                continue

            q_near = self.near(q_new, q_near, self.Δq*2)

            log.debug('get a new q %s', q_new)
            self.rrtree.add_node(
                q_new, q_near,
                self.distance_cost(q_new, q_near) + self.rrtree.vertexes[q_near])

            # We have reached the goal if the distance less than a given threshold
            if self.distance_cost(q_new, self.goal) < self.Δq:
                log.info('Finally we reach the goal with %s iteration!', k)
                if q_new != self.goal:
                    self.rrtree.add_node(self.goal, q_new, 1)
                return

    def near(self, q_new, q_near, delta):
        neighbors = {}

        for q in self.rrtree.vertexes.keys():
            line_cost = self.distance_cost(q, q_new)
            if line_cost < delta and self.path_obstruct_free(q, q_new):
                neighbors[q] = line_cost

        if not neighbors:
            return q_near

        log.info("star optimization start with %s neighbors....", len(neighbors))

        # neighbor -> cost from root
        ng_cost_list = [
            (q, c + self.rrtree.vertexes[q]) for q, c in neighbors.items()]

        # select least cost parent
        qx, x_cost = min(ng_cost_list, key=lambda e: e[1])

        # rewiring tree
        for ng, line_cost in ng_cost_list:
            if ng == qx:  # skip new parent
                continue

            log.info('try to rewiring tree...')
            if line_cost + x_cost < self.rrtree.vertexes[ng]:
                self.rrtree.path[ng] = qx
                self.rrtree.vertexes[ng] = line_cost + x_cost
                log.info('success rewire %s -> %s', ng, qx)

        return qx

    def find_path(self):
        if self.goal not in self.rrtree.path:
            return

        path = []
        cur = self.goal  # start with the last position (that is the goal)
        while True:
            q = self.rrtree.path[cur]
            path.insert(0, q)
            if q is None:
                break
            cur = q
        return path
