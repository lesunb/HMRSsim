from functools import reduce
from components.KitchenLayout import KitchenLayout, OpenOrder
from systems.KitchenManagement import CompleteOrderEventTag, CompleteOrderEventPayload
from components.Menu import Menu
from simulator.typehints.component_types import EVENT
from simulator.typehints.dict_types import SystemArgs
from simpy import Event

MAX_DISHES = 100


def cook(kwargs: SystemArgs):
    env = kwargs['ENV']
    event_store = kwargs['EVENT_STORE']
    kitchen_layout = kwargs['WORLD'].component_for_entity(1, KitchenLayout)
    menu = kwargs['WORLD'].component_for_entity(1, Menu)
    stove = kitchen_layout.areas['Stoves']
    table = kitchen_layout.areas['Assembly tables']
    orders = kwargs['EVENT_STORE']
    while True:
        if reduce(lambda acc, item: acc or (type(ord) == OpenOrder and item.stove), orders.items, False) and \
           stove.resource.count - stove.resource.capacity >= 2:
            order = yield orders.get(lambda ord: type(ord) == OpenOrder and ord.stove)
            with stove.resource.request() as req:
                yield req
                yield env.timeout(3)   # Using stove adds 3min to total time
        else:
            order = yield orders.get(lambda ord: type(ord) == OpenOrder and not ord.stove)
        # Some time to get the ingredients
        recipes = list(map(lambda x: menu.items[x], order.items))
        ingredient_count = reduce(lambda acc, recipe: acc + len(recipe.ingredients), recipes, 0)
        # Let's say it takes 30s + (10s * number ingredients) to get everything + 20s if there are more than 20
        # Because you have to take 2 trips. And remove 5s for cummulative recipes just because
        ingredients_time = 0.5 + (0.1 * ingredient_count) + ((ingredient_count // 20) * 0.2)
        yield env.timeout(ingredients_time)
        with table.resource.request() as req:
            yield req
            # We add the recipes prep_time and shave 30s off each because cooks are very efficient
            # When multitasking (so only if there's more than 1 recipe)
            prep_time = reduce(lambda acc, item: acc + item.prep_time - 0.5, recipes, 0)
            yield env.timeout(prep_time + 0.5)  # +10 compensate for the 1st recipe
        # Order is complete!
        payload = CompleteOrderEventPayload(order.orderId, env.now - order.createdAt)
        event_store.put(EVENT(CompleteOrderEventTag, payload))


def process(kwargs: SystemArgs):
    world = kwargs['WORLD']
    env = kwargs['ENV']
    kitchen_layout = world.component_for_entity(1, KitchenLayout)
    for _ in range(kitchen_layout.personnel):
        env.process(cook(kwargs))
    yield Event(env)

