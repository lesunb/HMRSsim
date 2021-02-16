from components.Path import Path
from systems.MovementProcessor import MovementProcessor
from systems.PathProcessor import PathProcessor
from simulator.main import Simulator
from components.Position import Position

config = {
 "context": "resources/map",
 "FPS": 60,
 "DLW": 10,
 "duration": 10
 }


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