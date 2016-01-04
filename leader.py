from boid import Boid
from vector2 import Vector2


class Leader(Boid):

    def __init__(self, x, y):
        super(Leader, self).__init__(x, y)
        self.velocity = Vector2(60, 60)

    def _set_speed(self, s):
        pass

    def update(self, t):
        """
        Velocity never changes for leaders.
        """
        self.position += self.velocity * t
