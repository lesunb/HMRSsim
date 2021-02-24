from components.Position import Position
from components.Collision import Collision
from components.Path import Path
from main import EVENT
from systems.GotoDESProcessor import GotoPosEventTag, GotoPosPayload
import json
import systems.SeerPlugin as Seer

def get_entity_id(simulation, drawio_object_id):
    """Gets the simulation entity id given the id of the drawio object"""
    drawio_object_id = str(drawio_object_id)

    for obj in simulation.objects:
        if obj[1] == drawio_object_id:
            return obj[0]
    return None

def get_style_id(simulation, drawio_style_id):
    id = simulation.draw2ent[str(drawio_style_id)][0]    
    return id

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
    x = pos[0]
    y = pos[1]
    payload = GotoPosPayload(id, [x, y])
    new_event = EVENT(GotoPosEventTag, payload)
    event_store = simulation.KWARGS['EVENT_STORE']
    event_store.put(new_event)

def store_collision_event(simulation, entity_id):
    id = get_entity_id(simulation, entity_id)

def get_style_component(simulation, component, drawio_id):
    id = get_style_id(simulation, drawio_id)
    component = simulation.world.component_for_entity(id, component)
    return component

# não esta funcionando com object
def get_center(simulation, drawio_id):
    position = get_style_component(simulation, Position, drawio_id)
    return position.center

def create_path(simulation, robot_id, points):
    path = Path(points)
    add_component(simulation, path, robot_id)

def have_collided(simulation, entity_id, other_entity_id):
    try:
        collision = get_component(simulation, Collision, entity_id) # TODO: levando em consideração que apenas uma colisão ocorreu
    except Exception:
        return False
    
    if collision.other_entity == get_style_id(simulation, other_entity_id):  # TODO: não está levando em consideraçao colisao entre dois objetos 
        return True
    return False
    