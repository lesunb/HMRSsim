from simpy import Environment
from esper import World

from components.Skeleton import Skeleton
from components.Position import Position

def process(kwargs):
    event_store = kwargs.get('EVENT_STORE', None)
    world: World = kwargs.get('WORLD', None)
    env: Environment = kwargs.get('ENV', None)
    if event_store is None:
        raise Exception("Can't find eventStore")
    elif env is None:
        raise Exception("Can't find env")

    while True:
        yield env.timeout(5)
        print(f'{env.now} SeerPlugin| Another round of information')
        skeleton = world.component_for_entity(2, Skeleton)
        position = world.component_for_entity(2, Position)

        data = {
            'id': skeleton.id,
            'value': skeleton.value,
            'x': position.x,
            'y': position.y,
            'width': position.w,
            'height': position.h,
            'style': skeleton.style
        }
        print(f'{env.now} SeerPlugin| Data for ent {2}: {data}')
