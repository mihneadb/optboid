#
# This is where the fun stuff happens - simulating the boids
#
# This file includes Vector classes borrowed from Cocos2D
#
# This library is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published by the
# Free Software Foundation; either version 2.1 of the License, or (at your
# option) any later version.
#
# This library is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License
# for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this library; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

from __future__ import division, print_function, absolute_import, unicode_literals

import random

from boid import Boid
from boid_swarm import BoidSwarm


class Simulation(object):

    """
    Ties the BoidSwarm with the boids.
    boidswarm just holds the data, boids know how to interact with each
    other. This class keeps the two separated
    """

    def __init__(self, starting_units=100, field_size=800):
        """
        """
        self.swarm = BoidSwarm(field_size+2*40, Boid.influence_range+5)  # /2
        self.field_size = field_size
        self.pad = 40  # use to keep boids inside the play field

        for i in range(starting_units):
            b = Boid(random.uniform(100, 400),
                     random.uniform(100, 400))
            self.swarm.boids.append(b)
        self.swarm.rebuild()
        self._cumltime = 0  # calculation var

    def update(self, dt):
        """dt is in seconds"""
        for b in self.swarm.boids:
            close_boids = self.swarm.find_near(b.position.x, b.position.y, b.influence_range)
            b.interact(close_boids)
            b.update(dt)
            w = self.field_size
            p = self.pad
            b.borders(p, w-p, p, w-p)  # keep the boids inside the borders

        # rebuild the swarm once we've updated all the positions
        self.swarm.rebuild()
