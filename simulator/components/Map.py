from components.Path import Path
from typehints.component_types import Component


class Map(Component):

    def __init__(self, paths={}):
        self.paths = paths
