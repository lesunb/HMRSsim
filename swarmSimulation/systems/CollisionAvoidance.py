from typing import List

import esper
import logging

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
        find_safe_route(hover, mypos, myvel, they)


def find_safe_route(hover: Hover, mypos: Position, myvel: Velocity, they: List[Position]):
    is_hovering = hover.status == HoverState.HOVERING
    SAFE_FACTOR = 3 if not is_hovering else 0
    # 10 degrees increments from original goal up do 90 degrees
    ROUTES = [
        0,
        0.1745329, -0.1745329, 0.3490658, -0.3490658, 0.5235987, -0.5235987,
        0.6981316, -0.6981316, 0.8726645, -0.8726645, 1.0471974, -1.0471974,
        1.2217303, -1.2217303, 1.3962632, -1.3962632, 1.5707961, -1.5707961
    ]
    for i in ROUTES:
        newx = mypos.x - SAFE_FACTOR + myvel.x
        newy = mypos.y - SAFE_FACTOR + myvel.y
        newx, newy = rotate_around_point((newx, newy), i, (mypos.x, mypos.y))
        my_next_position = Position(
            x=newx,
            y=newy,
            w=mypos.w + SAFE_FACTOR,
            h=mypos.h + SAFE_FACTOR
        )
        hit = False
        for other in they:
            if intercept(my_next_position, other):
                hit = True
                break
        if not hit:
            myvel.x = ((newx + SAFE_FACTOR) - mypos.x) / (1 + len(they) if not is_hovering else 1.1)
            myvel.y = ((newy + SAFE_FACTOR) - mypos.y) / (1 + len(they) if not is_hovering else 1.1)
            return
    myvel.x = 0 if not is_hovering else myvel.x / 1.5
    myvel.y = 0 if not is_hovering else myvel.x / 1.5


def intercept(a, b):
    if a.x + a.w < b.x or b.x + b.w < a.x:
        return False
    elif a.y + a.h < b.y or b.y + b.w < a.y:
        return False
    return True
