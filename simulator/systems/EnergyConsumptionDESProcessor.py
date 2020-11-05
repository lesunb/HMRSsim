from typing import NamedTuple

from simpy import Environment
from esper import World

from components.BatteryComponent import Battery

# Clock independente do tick da simulação - 1s
# Buscar no mundo as entidades que tem componente Battery
# Pra cada entidade:
#    -- Pegar a tabela de consumo da entidade E a ação dela atualmente
#    -- Calcular o gasto de energia no ultimo s
#    -- Atualizar a energia que sobrou

CHANGE_ACTION_PAYLOAD = NamedTuple('ChangeActionPayload', [('entity', int), ('newAction', str)])
CHANGE_ACTION_TAG = 'ChangeActionEvent'


def process(kwargs):
    event_store = kwargs.get('EVENT_STORE', None)
    world: World = kwargs.get('WORLD', None)
    env: Environment = kwargs.get('ENV', None)
    if event_store is None:
        raise Exception("Can't find eventStore")
    if env is None:
        raise Exception("Can't find env")

    while True:
        item = event_store.get(lambda ev: ev.type == CHANGE_ACTION_TAG)
        timeout = env.timeout(1)
        event = yield item | timeout
        if item in event:
            # Event
            item = event[item]
            payload = item.payload
            bat = world.component_for_entity(payload.entity, Battery)
            bat.currentAction = payload.newAction if payload.newAction in bat.lookupTable else 'default'
            print(f'Current action of entity {payload.entity} is {bat.currentAction}')
        if timeout in event:
            for ent, (bat,) in world.get_components(Battery):
                delta = bat.lookupTable.get(bat.currentAction, None)
                if delta is None:
                    delta = bat.lookupTable['default']
                bat.charge -= delta
