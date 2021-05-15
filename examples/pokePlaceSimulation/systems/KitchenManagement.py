import logging

from typing import NamedTuple, List
from enum import Enum
from functools import reduce
from simpy import Resource, FilterStore

from simulator.typehints.component_types import EVENT
from typehints.dict_types import SystemArgs

from examples.pokePlaceSimulation.components.KitchenLayout import KitchenLayout, OpenOrder
from examples.pokePlaceSimulation.components.Menu import Menu


class OrderOrigin(Enum):
    IN_HOUSE = 'in house',
    INTERNET = 'internet',
    PHONE = 'phone'


IncomingOrderEventTag = 'IncomingOrder'
IncomingOrderEventPayload = NamedTuple('IncomingOrder', [('origin', OrderOrigin), ('order', List[str])])

CompleteOrderEventTag = 'CompleteOrder'
CompleteOrderEventPayload = NamedTuple('CompleteOrder', [('order', int), ('total_time', float)])


def filter_function(event: EVENT):
    return type(event) != OpenOrder and event.type in [IncomingOrderEventTag, CompleteOrderEventTag]


def process(kwargs: SystemArgs):
    event_store = kwargs['EVENT_STORE']
    env = kwargs['ENV']
    kitchen_layout = kwargs['WORLD'].component_for_entity(1, KitchenLayout)
    # Creates a resource to manage cooks concurrently
    kitchen_layout.available_cooks = Resource(env, kitchen_layout.personnel)
    # For areas that have a max_capacity, create a resource to manage it
    for k, v in kitchen_layout.areas.items():
        if v.max_resources > 0:
            v.resource = Resource(env, v.max_resources)
    # Sink area needs an extra info - amount of dishes piling up
    kitchen_layout.areas['Sink'].dishes = 0
    handlers = {
        IncomingOrderEventTag: handle_incoming_order,
        CompleteOrderEventTag: handle_complete_order
    }
    while True:
        event = yield event_store.get(filter_function)
        handlers[event.type](event.payload, kwargs)


def handle_incoming_order(payload: IncomingOrderEventPayload, kwargs: SystemArgs):
    kitchen_layout = kwargs['WORLD'].component_for_entity(1, KitchenLayout)
    menu = kwargs['WORLD'].component_for_entity(1, Menu)
    now = kwargs['ENV'].now
    event_store = kwargs['EVENT_STORE']
    stove = reduce(lambda acc, item: acc or menu.items[item].usesStove, payload.order, False)
    new_order = OpenOrder(kitchen_layout.next_order, now, stove, payload.order)
    event_store.put(new_order)
    kitchen_layout.next_order += 1


def handle_complete_order(payload: CompleteOrderEventPayload, kwargs: SystemArgs):
    kitchen_layout = kwargs['WORLD'].component_for_entity(1, KitchenLayout)
    now = kwargs['ENV'].now
    kitchen_layout.orders_processed += 1
    kitchen_layout.avg_time_per_order = (kitchen_layout.avg_time_per_order * 0.4) + (payload.total_time * 0.6)
    # print(f'[{now}] Completed order {payload.order} in {payload.total_time}s')
