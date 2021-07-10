from collections import namedtuple
from typing import Dict, List
import simpy

from simulator.typehints.component_types import Component, Point
ControlResponseFormat = namedtuple('ControlResponse', ['drone', 'success'])


class Control(Component):
    def __init__(self, configs=Dict[str, List[Point]], channel=simpy.Store):
        self.configs = configs
        self.channel = channel
        self.awaiting = 0
        self.success = 0
        self.error = 0

    def __str__(self):
        return f'Control[configs={self.configs.keys()}]'
