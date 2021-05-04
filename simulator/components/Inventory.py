from simulator.typehints.component_types import Component


class Inventory(Component):

    def __init__(self, objects=None):
        if objects is None:
            objects = {}
        self.objects = objects
