from typing import List
from collision import Poly

from typehints.component_types import Component, Point, ShapeDefinition
from utils.helpers import tuple2vector, get_rel_points


class Collidable(Component):
    def __init__(self, shape_definitions: List[ShapeDefinition], collision_tag='genericCollision'):
        self.shapes = []
        for s in shape_definitions:
            self.shapes.append(Poly(tuple2vector(s[0]), get_rel_points(s[0], s[1])))

        self.event_tag = collision_tag

    def __str__(self):
        return f"Collidable[{len(self.shapes)} shapes. Tag={self.event_tag}]"
