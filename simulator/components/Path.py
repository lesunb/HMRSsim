from typehints.component_types import Component
import utils.helpers as helpers


class Path(Component):
    def __init__(self, points, speed=5):
        self.points = points
        self.curr_point = 0
        self.speed = speed

    def __str__(self):
        return f"Path{self.points} at point {self.curr_point}"
