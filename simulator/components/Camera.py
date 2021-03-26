from typehints.component_types import Component


class Camera(Component):

    def __init__(self, radius=100):
        self.detected_entities = []
        self.radius = radius

    def __str__(self):
        return f'Camera[detected_entities={self.detected_entities}]'
