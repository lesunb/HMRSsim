from typehints.component_types import Component

class Detectable(Component):

    def __init__(self, detectable=True):
        self.detectable = detectable

    def __str__(self):
        return f'Detectable[detectable={self.detectable}]'
