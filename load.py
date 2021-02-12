from components.Path import Path
from components.Position import Position
from main import Simulator
from systems.PathProcessor import PathProcessor
from systems.MovementProcessor import MovementProcessor
from tests.aux.testaux import get_component, get_path_last_point

config = {
        "context": "tests/data",
        "FPS": 60,
        "DLW": 10,
        "duration": 10
    }

config["map"] = "room3.drawio"
simulation = Simulator(config)

#buscar a posicao final da seta
path = get_component(simulation, Path, 'robot')
target_path = get_path_last_point(path)
print(path.points)
print(target_path)
print(path)

width, height = simulation.window_dimensions
simulation.add_system(PathProcessor())
simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))
simulation.run()

#buscar a posicao do robo
position = get_component(simulation, Position, 'robot')
print(position.center)
print(path)