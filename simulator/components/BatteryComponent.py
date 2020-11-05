
class Battery:

    def __init__(self, charge, lookupTable=None):
        self.charge = charge
        self.currentAction = 'still'
        self.lookupTable = lookupTable

    def __str__(self):
        return f'Battery[charge={self.charge}]'
