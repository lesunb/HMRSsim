from typehints.component_types import Component


class Collision(Component):
    def __init__(self, other_entity, time, position):
        self.collisions = {}
        self.add_collision(other_entity, time, position)

    def add_collision(self, other_entity, time, position):
        if other_entity not in self.collisions:
            self.collisions[other_entity] = [time, position]

    def __str__(self):
        return f"Collisions: {self.collisions}"