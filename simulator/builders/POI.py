import esper
import logging
from components.Map import Map
from components.Skeleton import Skeleton
from components.Position import Position

TYPE = 'POI'


def build_object(object, world: esper.World, window_options, draw2entity):
    logger = logging.getLogger(__name__)
    mxCell = object[0]
    mxGeometry = mxCell[0]
    # Get X, Y coordinates
    x = float(mxGeometry.attrib.get('x', 0))
    y = float(mxGeometry.attrib.get('y', 0))
    width = float(mxGeometry.attrib.get('width', 0))
    height = float(mxGeometry.attrib.get('height', 0))
    pos = Position(x, y, 0, width, height)
    x += (width // 2)
    y += (height // 2)
    # Get the Map for simulation or create one
    # Entity 1 is the simulation entity
    if world.has_component(1, Map):
        simulation_map = world.component_for_entity(1, Map)
    else:
        simulation_map = Map()
        world.add_component(1, simulation_map)
    # Get POI tag
    if 'tag' in object.attrib:
        tag = object.attrib['tag']
    else:
        tag = 'POI_' + str(len(simulation_map.pois))
        logger.warning(f'POI ({x}, {y}) with no TAG. Using {tag}')
    simulation_map.pois[tag] = (x, y)
    # Alternatively display the POI
    if object.attrib.get('display', False):
        skeleton = Skeleton(object.attrib['id'], mxCell.attrib['style'], tag)
        world.create_entity(pos, skeleton)
    return {}, [], {}
