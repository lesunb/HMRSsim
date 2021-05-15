
from esper import World
from typing import Tuple
from examples.pokePlaceSimulation.components.KitchenArea import KitchenArea
from examples.pokePlaceSimulation.components.KitchenLayout import KitchenLayout

TYPE = 'kitchen_area'


def build_object(cell, world: World, *args) -> Tuple[dict, list, dict]:
    area_name = cell.attrib['label']
    max_resources = cell.attrib.get('max_resources', 0)
    geometry = cell[0][0]
    x = geometry.attrib['x']
    y = geometry.attrib['y']
    kitchen_area = KitchenArea(area_name, int(max_resources), (float(x), float(y)))
    kitchen_layout = world.component_for_entity(1, KitchenLayout)
    kitchen_layout.areas[area_name] = kitchen_area
    return {}, [], {}
