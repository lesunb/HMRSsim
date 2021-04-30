from simulator.typehints.component_types import Component


class CollisionHistory(Component):
    def __init__(self):
        """The component keeps all the collisions of one entity."""
        self.collisions = {}

    def add_collision(self, other_entity, time, position):
        self.collisions[other_entity] = [time, position]

    def __str__(self):
        return f"Collisions: {self.collisions}"