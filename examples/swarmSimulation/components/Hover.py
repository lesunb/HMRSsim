from simulator.typehints.component_types import Component, Point
import enum


class HoverState(enum.Enum):
    HOVERING = '#2e8B57',
    MOVING = '#ffd700',
    LANDED = '#708090',
    CRASHED = '#ff0000',


class Hover(Component):

    def __init__(self,
                 status: HoverState = HoverState.LANDED,
                 target: Point = None):
        self.status = status
        self.target = target
        self.crowded = []

    def __str__(self):
        return f'Hover[status: {self.status}; target: {self.target}]'
