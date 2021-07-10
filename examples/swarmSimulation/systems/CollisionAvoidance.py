from typing import List

import esper
import random

from functools import reduce
from simulator.components.ProximitySensor import ProximitySensor
from simulator.components.Position import Position
from simulator.components.Velocity import Velocity
from components.Hover import Hover, HoverState

from simulator.utils.helpers import rotate_around_point


def find_safe_route(hover: Hover, mypos: Position, myvel: Velocity, they: List[Position]):
    # Local refs to functions
    rotate_point = rotate_around_point
    pos_x = mypos.x
    pos_y = mypos.y
    #
    is_hovering = hover.status == HoverState.HOVERING
    SAFE_FACTOR = 1 if not is_hovering else 0
    # 10 degrees increments from original goal up do 90 degrees
    HEADINGS = [
        0.0, 0.523599, 1.047198, 1.48353,
        5.23599, 5.759589, 5.759587,
        2.094396, 2.617995, 3.061594, 3.665193,
        4.188792, 4.712391
    ]
    hits = [(0, 0, 0, 0)] * 13
    for i, h in enumerate(HEADINGS):
        newx = pos_x + myvel.x
        newy = pos_y + myvel.y
        newx, newy = rotate_point((newx, newy), h, (pos_x, pos_y))
        my_next_position = Position(
            x=newx,
            y=newy,
            w=mypos.w,
            h=mypos.h
        )
        hit = reduce(
            lambda acc, hit_count: acc + hit_count,
            map(lambda t: intercept(my_next_position, t, SAFE_FACTOR), they),
            0
        )
        # for other in they:
        #     if intercept(my_next_position, other, SAFE_FACTOR):
        #         hit += 1
        hits[i] = (hit, i, newx, newy)
    hits.sort()
    (hit, i, newx, newy) = hits[0]
    myvel.x = (newx - pos_x) / (1 + hit + random.random() if not is_hovering else 1.1)
    myvel.y = (newy - pos_y) / (1 + hit + random.random() if not is_hovering else 1.1)


def intercept(a, b, safe_factor):
    if a.x + a.w < (b.x - safe_factor) or (b.x + b.w + safe_factor) < a.x:
        return 0
    elif a.y + a.h < (b.y - safe_factor) or (b.y + b.w + safe_factor) < a.y:
        return 0
    return 1
