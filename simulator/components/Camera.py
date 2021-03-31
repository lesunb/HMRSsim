from typehints.component_types import Component


class Camera(Component):

    def __init__(self, range=100):
        self.detected_entities = {}
        self.range = range
        self.reply_channel = None

    def __str__(self):
        return f'Camera[detected_entities={self.detected_entities}]'
