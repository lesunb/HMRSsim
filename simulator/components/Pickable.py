from simulator.typehints.component_types import Component


class Pickable(Component):

    def __init__(self, weight, name, skeleton):
        self.weight = weight
        self.name = name
        self.skeleton = skeleton
