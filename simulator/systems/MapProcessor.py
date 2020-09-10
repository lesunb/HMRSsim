from typing import NamedTuple

from components.Map import Map
from components.Path import Path

MAP_PAYLOAD = NamedTuple('MapPayload', [('entity', int), ('route', str)])
MAP_EVENT_TAG = 'MapEvent'

def process(kwargs):
    event_store = kwargs.get('EVENT_STORE', None)
    world = kwargs.get('WORLD', None)
    if event_store is None:
        raise Exception("Can't find eventStore")

    while True:
        # Gets next map event
        # TODO: Verify if entity is close to path start
        event = yield event_store.get(lambda ev: ev.type == MAP_EVENT_TAG)
        payload: MAP_PAYLOAD = event.payload
        entity_map = world.component_for_entity(payload.entity, Map)
        path_to_follow = entity_map.paths.get(payload.route, None)
        if path_to_follow is None:
            print(f"Path {payload.route} not found!")
            continue
        path_component = Path(points=path_to_follow)
        # print(f"Path to follow is {path_to_follow} ({isinstance(path_component, Path)})")
        print(f"Adding path {payload.route} to entity {payload.entity}")
        world.add_component(payload.entity, path_component)
