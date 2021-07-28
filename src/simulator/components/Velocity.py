from simulator.typehints.component_types import Component


class Velocity(Component):
    def __init__(self, x=0.0, y=0.0, alpha=0.0):
        self.x = x
        self.y = y
        self.alpha = alpha

    def __str__(self):
        return f"Velocity[x={self.x}, y={self.y}, alpha={self.alpha}]"
