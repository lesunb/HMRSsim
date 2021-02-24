from typehints.dict_types import SystemArgs
from components.Collision import Collision

def process(kwargs: SystemArgs):
    event_store = kwargs.get('EVENT_STORE', None)
    world = kwargs.get('WORLD', None)
    if event_store is None:
        raise Exception("Can't find eventStore")
    #while True: # busca apenas uma colisão
    event = yield event_store.get(lambda ev: ev.type == 'stopEvent')
    payload = event.payload
    entity_id = payload[0]
    collided = Collision(payload[1])
    world.add_component(entity_id, collided)

    #print("entity: ", payload)
        