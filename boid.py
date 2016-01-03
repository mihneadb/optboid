import random
from math import cos, atan2
from math import sin
from vector2 import Vector2


def limit(vector, lim):
    """
    limit a vector to a given magnitude
    this is an 'in place' function, modifies the vector supplied.
    """
    if abs(vector) > lim:
        vector.normalize()
        vector *= lim


class Boid(object):

    """
    Boids class
    """
    influence_range = 90
    minsep = 25.0
    max_force = 20.0
    max_speed = 180.0
    drag = 0.9
    cohesion_strength = 1.5
    align_strength = 1.4
    sep_strength = 1.0

    # cohesion_strength *=  max_force
    # align_strength *=  max_force
    # sep_strength *= max_force

    # Get and set the speed as a scalar
    def _get_speed(self):
        return abs(self.velocity)

    def _set_speed(self, s):
        if abs(self.velocity) == 0:
            self.velocity = Vector2(1, 0)

        self.velocity.normalize()
        self.velocity *= s

    speed = property(_get_speed, _set_speed)

    # get and set the rotation as an angle
    def _get_rotation(self):
        return atan2(self.velocity.y, self.velocity.x)

    def _set_rotation(self, r):
        old_speed = self.speed
        # set the direction as a unit vector
        self.velocity.x = cos(r)
        self.velocity.y = sin(r)

        self.speed = old_speed

    rotation = property(_get_rotation, _set_rotation)

    def __init__(self, x, y):
        """ create a new boid at x,y """
        self.neighbors = 0

        self.position = Vector2(x, y)
        self.acceleration = Vector2(0, 0)
        self.velocity = Vector2(random.uniform(-self.max_speed, self.max_speed),
                                random.uniform(-self.max_speed, self.max_speed))

    def __repr__(self):
        return 'id %d' % self.id

    def borders(self, top, bottom, left, right):
        """
        Cycle boids when going out of bounds.
        """
        if self.position.x < left:
            self.position.x = right
        if self.position.x > right:
            self.position.x = left
        if self.position.y < top:
            self.position.y = bottom
        if self.position.y > bottom:
            self.position.y = top

    def update(self, t):
        """
        Method to update position by computing displacement from velocity and acceleration
        """
        self.velocity += self.acceleration * t
        limit(self.velocity, self.max_speed)
        self.position += self.velocity * t

    # Calculation variables for interact method - init once instead of on each call
    _sep_f = Vector2(0, 0)
    _align_f = Vector2(0, 0)
    _cohes_sum = Vector2(0, 0)

    def interact(self, actors):
        """
        Unit-unit interaction method, combining a separation force, and velocity
        alignment force, and a cohesion force.

        Many examples separate these into different functions for clarity
        but combining them means we need fewer loops over the neibor list
        """

        self._sep_f.clear()
        self._align_f.clear()
        self._cohes_sum.clear()

        count = 0
        self.neighbors = len(actors)

        for other in actors:
            # vector pointing from neighbors to self
            diff = self.position - other.position
            d = abs(diff)

            # Only perform on "neighbor" actors, i.e. ones closer than arbitrary
            # dist or if the distance is not 0 (you are yourself)
            if 0 < d < self.influence_range:
                count += 1

                diff.normalize()
                if d < self.minsep:
                    diff /= d  # Weight by distance
                self._sep_f += diff

                self._cohes_sum += other.position  # Add position

                # Align - add the velocity of the neighbouring actors, then average
                self._align_f += other.velocity

        if count > 0:
            # calc the average of the separation vector
            # self._sep_f /=count don't div by count if normalizing anyway!
            self._sep_f.normalize()
            self._sep_f *= self.max_speed
            self._sep_f -= self.velocity
            limit(self._sep_f, self.max_force)

            # calc the average direction (normed avg velocity)
            # self._align_f /= count
            self._align_f.normalize()
            self._align_f *= self.max_speed
            self._align_f -= self.velocity
            limit(self._align_f, self.max_force)

            # calc the average position and calc steering vector towards it
            self._cohes_sum /= count
            cohesion_f = self.steer(self._cohes_sum, True)

            self._sep_f *= self.sep_strength
            self._align_f *= self.align_strength
            cohesion_f *= self.cohesion_strength

            # finally add the velocities
            sum = self._sep_f + cohesion_f + self._align_f

            self.acceleration = sum

    def steer(self, desired, slowdown=False):
        """
        A helper method that calculates a steering vector towards a target
        If slowdown is true the steering force is reduced as it approaches the target
        """
        desired -= self.position
        d = abs(desired)
        # If the distance is greater than 0, calc steering (otherwise return zero vector)
        if d > 0:
            desired.normalize()
            if slowdown and (d < self.minsep):
                desired *= self.max_speed*d / self.minsep
            else:
                desired *= self.max_speed

            steer = desired - self.velocity
            limit(steer, self.max_force)
        else:
            steer = Vector2(0, 0)
        return steer
