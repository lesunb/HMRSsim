from main import Simulator

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

"""
config = {
        "context": "tests/data",
        "map": "room10x10.drawio",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }

from components.Position import Position

simulation = Simulator('tests/data/test_simulation.json')
position = get_component(simulation, Position, '6')
position.x = 1
position.y = 1

simulation.run()
"""