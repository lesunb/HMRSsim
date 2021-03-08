from components.Position import Position
from components.Collision import Collision
from components.Path import Path
from components.Map import Map
from typehints.component_types import EVENT
from systems.GotoDESProcessor import GotoPoiEventTag, GotoPoiPayload, GotoPosEventTag, GotoPosPayload

def get_object_id(simulation, drawio_id):
    """Gets the simulation entity id given the id of the drawio object."""
    drawio_id = str(drawio_id)

    for obj in simulation.objects:
        if obj[1] == drawio_id:
            return obj[0]
    return None

def get_drawio_id(simulation, drawio_id):
    entity_id = simulation.draw2ent.get(drawio_id, None)  
    if entity_id:
        return entity_id[0]
    return entity_id

def get_pickable_id(simulation, name):
    return simulation.interactive.get(name, None)

def cast_id(simulation, drawio_id):
    """Converts the object id present in the xml tree to the object id in the simulation.
    Tipos possíveis:
    - No caso do objeto ser do tipo pickable, passar a propriedade name como
    parametro para o drawio_id
    """
    entity_id = get_object_id(simulation, drawio_id)
    if entity_id:
        return entity_id
    entity_id = get_pickable_id(simulation, drawio_id) 
    if entity_id:
        return entity_id   
    return get_drawio_id(simulation, drawio_id)

def get_component(simulation, component, id):
    entity_id = cast_id(simulation, id)
    if simulation.world.has_component(entity_id, component):
        return simulation.world.component_for_entity(entity_id, component)
    else:
        return None

def get_position(simulation, drawio_id):
    return get_component(simulation, Position, drawio_id)

def add_component(simulation, component, drawio_object_id):
    id = cast_id(simulation, drawio_object_id)
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
    
def get_poi(simulation, poi_tag):
    map = simulation.world.component_for_entity(1, Map)
    return map.pois[poi_tag]

def is_in_poi(simulation, entity_id, poi_tag):
    entity_position = get_position(simulation, entity_id)  # verificar se é para pegar o centro
    poi = get_poi(simulation, poi_tag)

    if entity_position.x == poi[0] and entity_position.y == poi[1]:
        return True
    else:
        return False
        #raise AssertionError(f'Entity is not in the POI. Entity Position: {entity_position}. Poi: {poi}')
        #como retornar o Assertion e o False

def is_in_center_of(simulation, entity_id, other_entity_id):
    entity_center = get_center(simulation, entity_id)
    other_entity_center = get_center(simulation, other_entity_id)

    if entity_center[0] == other_entity_center[0] and entity_center[1] == other_entity_center[1]:
        return True
    return False
