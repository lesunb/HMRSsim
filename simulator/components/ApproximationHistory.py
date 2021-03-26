from typehints.component_types import Component


class ApproximationHistory(Component):

    def __init__(self):
        self.approximations = []

    def __str__(self):
        return f'ApproximationHistory[approximations={self.approximations}]'
