from simulator.typehints.component_types import Component


class Claw(Component):

    def __init__(self, max_range, max_weight):
        self.max_range = int(max_range)
        self.max_weight = float(max_weight)

    def __str__(self):
        return f'Claw[range={self.max_range}, max_weight={self.max_weight}]'
