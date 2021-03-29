from typing import List

import esper
import random

from simulator.components.ProximitySensor import ProximitySensor
from simulator.components.Position import Position
from simulator.components.Velocity import Velocity
from swarmSimulation.components.Hover import Hover, HoverState

from simulator.utils.helpers import rotate_around_point


def dont_crash(world: esper.World, sensor: ProximitySensor):
    while True:
        ev = yield sensor.reply_channel.get()
        payload = ev.payload
        me = payload.ent
        mypos = payload.pos
        myvel = payload.vel
        they = list(map(lambda x: x.other_pos, payload.close_entities))
        hover = world.component_for_entity(me, Hover)
        hover.crowded = they
        # find_safe_route(hover, mypos, myvel, they)


def find_safe_route(hover: Hover, mypos: Position, myvel: Velocity, they: List[Position]):
    is_hovering = hover.status == HoverState.HOVERING
    SAFE_FACTOR = 1.5 if not is_hovering else 0
    # 10 degrees increments from original goal up do 90 degrees
    HEADINGS = [
        0.0, 0.523599, 1.047198, 1.48353,
        5.23599, 5.759589, 5.759587,
        2.094396, 2.617995, 3.061594, 3.665193,
        4.188792, 4.712391
    ]
    hits = [(0, 0, 0, 0)] * 13
    for i, h in enumerate(HEADINGS):
        newx = mypos.x + myvel.x
        newy = mypos.y + myvel.y
        newx, newy = rotate_around_point((newx, newy), h, (mypos.x, mypos.y))
        my_next_position = Position(
            x=newx,
            y=newy,
            w=mypos.w,
            h=mypos.h
        )
        hit = 0
        for other in they:
            if intercept(my_next_position, other, SAFE_FACTOR):
                hit += 1
        hits[i] = (hit, i, newx, newy)
    hits.sort()
    (hit, i, newx, newy) = hits[0]
    myvel.x = (newx - mypos.x) / (2 + hit if not is_hovering else 1.1)
    myvel.y = (newy - mypos.y) / (2 + hit if not is_hovering else 1.1)


def intercept(a, b, safe_factor):
    if a.x + a.w < (b.x - safe_factor) or (b.x + b.w + safe_factor) < a.x:
        return False
    elif a.y + a.h < (b.y - safe_factor) or (b.y + b.w + safe_factor) < a.y:
        return False
    return True
