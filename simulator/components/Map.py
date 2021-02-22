"""A map component holds navigation information that can be shared by robots in a team.

The simulator will have a Map component available.
Maps have nodes and pois (points of interest).
The Nodes can be used to find a suitable route to a specific position or poi.
"""
from typehints.component_types import Component, Point
from typing import List, Dict
from utils.Navigation import Node, POI


class Map(Component):

    def __init__(self, nodes: Dict[Point, Node] = {}, pois: List[POI] = [], point_width=20):
        self.nodes = nodes
        self.pois = pois
        self.point_width = point_width

    def __str__(self):
        return f'Map[{len(self.nodes)} nodes; {len(self.pois)} pois]\nNODES: {self.nodes}\nPOIS: {self.pois}'
