from typing import List, Tuple
from random import randint, sample, choice
from simulator.typehints.component_types import EVENT
from simulator.typehints.dict_types import SystemArgs
from components.Menu import Menu
from systems.KitchenManagement import IncomingOrderEventTag, IncomingOrderEventPayload, OrderOrigin


def init(stress: List[Tuple[int, int]]):
    def process(kwargs: SystemArgs):
        event_store = kwargs['EVENT_STORE']
        menu = kwargs['WORLD'].component_for_entity(1, Menu)
        env = kwargs['ENV']
        item_list = list(menu.items.keys())
        curr = 0
        while True:
            total_concurrent = stress[curr][0]
            curr_timeout = stress[curr][1]
            for i in range(total_concurrent):
                items = [choice(item_list) for _ in range(randint(1, 2))]
                origin = choice([OrderOrigin.PHONE, OrderOrigin.INTERNET, OrderOrigin.IN_HOUSE])
                payload = IncomingOrderEventPayload(origin, items)
                event_store.put(EVENT(IncomingOrderEventTag, payload))
            curr = (curr + 1) % len(stress)
            yield env.timeout(curr_timeout)

    return process
