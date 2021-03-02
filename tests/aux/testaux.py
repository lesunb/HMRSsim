from components.Position import Position
from components.Collision import Collision
from components.Path import Path
from main import EVENT
from systems.GotoDESProcessor import GotoPoiEventTag, GotoPoiPayload, GotoPosEventTag, GotoPosPayload

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

def cast_id(simulation, drawio_id):
    """Converts the object id present in the xml tree to the object id in the simulation."""
    drawio_id = str(drawio_id)

    for obj in simulation.objects:
        if obj[1] == drawio_id:
            return obj[0]

    id = simulation.draw2ent.get(drawio_id, None)
    if id:
        return id[0]
    return None

def get_component(simulation, component, id):
    entity_id = cast_id(simulation, id)
    if simulation.world.has_component(entity_id, component):
        return simulation.world.component_for_entity(entity_id, component)
    else:
        return None

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
    id = cast_id(simulation, entity_id)
    x = pos[0]
    y = pos[1]
    payload = GotoPosPayload(id, [x, y])
    new_event = EVENT(GotoPosEventTag, payload)
    event_store = simulation.KWARGS['EVENT_STORE']
    event_store.put(new_event)

def store_goto_poi_event(simulation, entity_id, poi_tag):
    id = cast_id(simulation, entity_id)
    payload = GotoPoiPayload(id, poi_tag)
    new_event = EVENT(GotoPoiEventTag, payload)
    event_store = simulation.KWARGS['EVENT_STORE']
    event_store.put(new_event)

def get_center(simulation, drawio_id):
    position = get_component(simulation, Position, drawio_id)
    return position.center

def create_path(simulation, robot_id, points):
    path = get_component(simulation, Path, robot_id)
    if path is None:
        path = Path(points)
        add_component(simulation, path, robot_id)
    else:
        for point in points:
            path.points.append(point)

def get_collision(simulation, entity_id, other_entity_id):
    collision = get_component(simulation, Collision, entity_id) 
    other_entity_id = cast_id(simulation, other_entity_id)
    
    if collision and other_entity_id in collision.collisions:  
        return collision.collisions[other_entity_id]
    return None

def have_collided(simulation, entity_id, other_entity_id):
    """Verifica se ocorreu 1 colisao"""
    collision = get_component(simulation, Collision, entity_id) 
    other_entity_id = cast_id(simulation, other_entity_id)
    
    if collision and other_entity_id in collision.collisions: 
        return True
    return False
    