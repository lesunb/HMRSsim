from typehints.component_types import Component


class Battery(Component):

    def __init__(self, charge, lookupTable=None):
        self.charge = charge
        self.currentAction = 'still'
        self.lookupTable = lookupTable

    def __str__(self):
        return f'Battery[charge={self.charge}]'
