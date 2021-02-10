from simpy.events import Event
from main import EVENT, Simulator
from systems.GotoDESProcessor import GotoPosEventTag, GotoPosPayload
from systems.PathProcessor import PathProcessor
#from systems.PathProcessor import PathProcessor
#from systems.MovementProcessor import MovementProcessor
from components.Position import Position

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
def store_gotoPos_event(simulation, entity_id, pos):
    id = get_entity_id(simulation, entity_id)
    payload = GotoPosPayload(id, pos)
    new_event = EVENT(GotoPosEventTag, payload)
    event_store = simulation.KWARGS['EVENT_STORE']
    event_store.push(new_event)

"""
config = {
        "context": "tests/data",
        "map": "room10x10.drawio",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }

from components.Position import Position

simulation = Simulator(config)
position = get_component(simulation, Position, '6')
position.x = 1
position.y = 1

simulation.run()
"""


from components.Path import Path
from systems.MovementProcessor import MovementProcessor
from systems.PathProcessor import PathProcessor
config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }

config["map"] = "room3.drawio"
simulation = Simulator(config)
print(simulation.objects)

width, height = simulation.window_dimensions
simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))
simulation.add_system(PathProcessor())
path = get_component(simulation, Path, 'robot')
simulation.run()
print(path)
position = get_component(simulation, Position, 'robot')
print(position.center)
"""
target = path.points[-1]
print(path.points)
path.points = path.points[0:-1]

#print(target[0])
#print(target[1])

position = get_component(simulation, Position, '15')
#print("Before")
#print(position)

width, height = simulation.window_dimensions
simulation.add_system(PathProcessor())
simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

simulation.run()
print("After")
print(position)
"""