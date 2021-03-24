from typehints.component_types import Component


class Camera(Component):

    def __init__(self, radius=100):
        self.captured_entities = []
        self.radius = radius

    def __str__(self):
        return f'Camera[captured_entities={self.captured_entities}]'
