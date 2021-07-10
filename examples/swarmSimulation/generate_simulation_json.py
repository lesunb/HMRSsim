import math
from typing import List

from simulator.typehints.component_types import Point, ShapeDefinition
from simulator.typehints.dict_types import Config, EntityDefinition, LogLevel

LOG_LEVEL = LogLevel.INFO

def collidable_from_position(pos: Point) -> List[ShapeDefinition]:
    center = (pos[0] + 2.5, pos[1] + 2.5)
    points = [
        (pos[0], pos[1]),
        (pos[0] + 5, pos[1]),
        (pos[0] + 5, pos[1] + 5),
        (pos[0], pos[1] + 5)
    ]
    return [(center, points)]


def create_drone(pos: List[int], drone_id: str) -> EntityDefinition:
    definition = EntityDefinition(
        entId='drone_' + drone_id,
        name='',
        isObject=True,
        isInteractive=False,
        type='drone',
        components= {
        "Position": [pos[0], pos[1], 0, 5, 5],
        "Collidable": [collidable_from_position((pos[0], pos[1]))],
        "Hover": [],
        "Skeleton": ['drone_' + drone_id, "rounded=0;whiteSpace=wrap;html=1;strokeColor=#FF0000;fillColor=#000000;"],
        "Velocity": [0, 0],
        "ProximitySensor": [8, 'drone_sensor']
    })
    return definition


def generate_simulation_json(drone_count: int):
    simulation = Config(
        context=".",
        map="swarm.drawio",
        FPS=24,
        DLW=10,
        verbose=LOG_LEVEL,
        extraEntities=[]
    )

    curr_position = [20, 20]
    distance = 20
    for i in range(drone_count):
        simulation['extraEntities'].append(create_drone(curr_position, str(i)))
        next_x = curr_position[0] + distance
        if next_x < 600:
            curr_position[0] += distance
        else:
            curr_position[0] = 20
            curr_position[1] += distance
    return simulation


def generate_ellipse(center: Point, a, b):
    points = []
    h = int(center[0])
    k = int(center[1])
    for i in range(h - a, h + a + 1, 15):
        y = math.sqrt(1 - ((i - h) / a) ** 2) * b + k
        points.append((i, y))
        points.append((i, k - (y - k)))
    return list(set(reversed(points)))


def generate_shapes(drone_count: int):
    center = (280, 200)
    curr_radius = 25
    points = generate_ellipse(center, curr_radius, curr_radius)
    while len(points) < drone_count:
        curr_radius += 25
        points += generate_ellipse(center, curr_radius, curr_radius)
    return {'CIRCLE': points}
