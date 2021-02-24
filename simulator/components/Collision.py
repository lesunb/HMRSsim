from typehints.component_types import Component


class Collision(Component):
    def __init__(self, other_entity):
        self.other_entity = other_entity

    def __str__(self):
        return f"Collided Entity: {self.other_entity}"
