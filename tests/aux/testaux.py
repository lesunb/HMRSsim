from main import EVENT
from systems.GotoDESProcessor import GotoPosEventTag, GotoPosPayload

def get_entity_id(simulation, drawio_object_id):
    """Gets the simulation entity id given the id of the drawio object"""
    drawio_object_id = str(drawio_object_id)

    for obj in simulation.objects:
        if obj[1] == drawio_object_id:
            return obj[0]
    return None

def get_component(simulation, component, drawio_object_id):
    id = get_entity_id(simulation, drawio_object_id)
    component = simulation.world.component_for_entity(id, component)
    return component

def add_component(simulation, component, drawio_object_id):
    id = get_entity_id(simulation, drawio_object_id)
    simulation.world.add_component(id, component)

def get_path_points(path):
    return path.points

def get_path_last_point(path):
    """Retorna um dict com as coordenadas x,y do ultimo ponto."""
    target_path = path.points[-1]
    return { 'x': target_path[0], 'y': target_path[1] }

# setar ponto para ir
def store_goto_position_event(simulation, entity_id, pos):
    id = get_entity_id(simulation, entity_id)
    payload = GotoPosPayload(id, pos)
    new_event = EVENT(GotoPosEventTag, payload)
    event_store = simulation.KWARGS['EVENT_STORE']
    event_store.put(new_event)
