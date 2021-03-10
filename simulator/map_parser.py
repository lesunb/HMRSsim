"""Parse of the drawio compressed XML into simulation entities and components.
"""
import esper
import logging
import os
import pathlib

import resources.load_resources as loader
import mxCellDecoder as mxCellDecoder

from dynamic_builders import export_available_builders
from utils.create_components import initialize_components
from components.Inventory import Inventory
from components.Skeleton import Skeleton
from xml.etree.ElementTree import Element
from typehints.build_types import WindowOptions, DependencyNotFound
from typing import List, Tuple


def build_simulation_from_map(file: pathlib.Path, skip_map=False, simulation_components=None, line_width=10):
    """Creates the base for the simulation.

        If a map is provided, the simulation comes from the map.
        Otherwise, and empty simulation is created.
    """
    logger = logging.getLogger(__name__)
    if os.path.isfile(file) and not skip_map:
        window_name, map_content = loader.map_from_drawio(file)
    else:
        if not skip_map:
            logger.error(f'Map file {file} does not exist. Creating empty simulation instead')
        window_name = 'Default'
        map_content = Element('', tag='not found', attrib={})

    width = int(map_content.attrib.get('pageWidth', 500))
    height = int(map_content.attrib.get('pageHeight', 500))

    background_color = '#FFFFFF'
    if 'background' in map_content.attrib:
        background_color = map_content.attrib['background']
    content_root = map_content[0] if len(map_content) > 0 else None
    # Create pyglet window
    # window = pyglet.window.Window(width=width,
    #                               height=height,
    #                               caption=window_name)
    # batch = pyglet.graphics.Batch()
    # Define default line width
    # pyglet.gl.glLineWidth(line_width)
    # Define background clear color
    # pyglet.gl.glClearColor(background_color[0], background_color[1], background_color[2], background_color[3])
    # pyglet.gl.glClear(pyglet.gl.GL_COLOR_BUFFER_BIT | pyglet.gl.GL_DEPTH_BUFFER_BIT)

    world = esper.World()
    simulation = world.create_entity(Inventory())  # Simulation is always the first entity
    if simulation_components is not None:
        initialized_components = initialize_components(simulation_components)
        for c in initialized_components:
            world.add_component(1, c)

    if content_root:
        context = (file.parent if not skip_map else file) / 'builders'
        available_builders = export_available_builders([context])
        draw_map, objects, interactive = build_simulation_objects(
            content_root,
            world,
            ((width, height), line_width),
            available_builders
        )
    else:
        draw_map = {}
        objects = []
        interactive = {}

    world.add_component(simulation, Inventory(interactive))
    skeleton_style = "{{\"width\":{:d},\"height\":{:d}}}".format(width, height)
    world.add_component(simulation, Skeleton(id=window_name, style=skeleton_style, model=True))
    return {
        'world': world,
        'window_props': (window_name, (width, height), background_color),
        'draw_map': draw_map,
        'objects': objects
    }


def build_simulation_objects(
        content_root: Element,
        world: esper.World,
        window_options: WindowOptions,
        available_builders: dict):
    """ Parses the XML Elements into esper entities.

        Parsing is done by transforming Element's attributes and annotations into components.
        There are 2 types of objects:
            1. Untyped objects -- Do not possess any type annotation. Used for building walls, for example.
                                  They have only basic components (e.g. Collision, Position, ...)
            2. Typed objects -- Possess a type annotation. Enclosed in <object> tags in the XML file.
                                Parsed by a specific builder according to the type.
                                Not all typed objects are transformed into entities.

        RETURNS:
            draw2entity: dict -- Maps ids from drawio to entities in the simulation.
            objects: list -- List of typed objects that have been transformed into entities.
            interactive: dict -- Maps what entities can be interacted with (e.g. picked up).
                                 Key is the entity name, value is the entity id
    """
    logger = logging.getLogger(__name__)
    draw2entity = {}
    objects: List[Tuple[int, str]] = []
    interactive = {}
    # If any object in the XML has dependencies that appear after it in the XML
    # The builder can return a DependencyNotFound error, causing the cell to be deferred.
    # Deferred cells are re-evaluated after the first pass is complete.
    deferred: List[Element] = []
    # 1st pass
    for cell in content_root:
        if cell.tag == 'mxCell' and 'style' in cell.attrib:
            (components, style) = mxCellDecoder.parse_mxCell(cell, window_options)
            ent = world.create_entity()
            for c in components:
                world.add_component(ent, c)
            draw2entity[style['id']] = [ent, style]
        if cell.tag == 'object':
            cell_type = cell.attrib['type']
            try:
                pending_updates = \
                    available_builders[cell_type].__dict__['build_object'](cell, world, window_options, draw2entity)
                draw2entity.update(pending_updates[0])
                objects += pending_updates[1]
                interactive.update(pending_updates[2])
            except DependencyNotFound as err:
                deferred.append(cell)
                logger.debug(f'Cell {cell.tag} deferred - {err}')
    # 2nd pass
    for cell in deferred:
        if cell.tag == 'mxCell' and 'style' in cell.attrib:
            (components, style) = mxCellDecoder.parse_mxCell(cell, window_options)
            ent = world.create_entity()
            for c in components:
                world.add_component(ent, c)
            draw2entity[style['id']] = [ent, style]
        if cell.tag == 'object':
            cell_type = cell.attrib['type']
            try:
                pending_updates = \
                    available_builders[cell_type].__dict__['build_object'](cell, world, window_options, draw2entity)
                draw2entity.update(pending_updates[0])
                objects += pending_updates[1]
                interactive.update(pending_updates[2])
            except DependencyNotFound as err:
                logger.error(f'Cell {cell.tag} failed processing - {err}')

    return draw2entity, objects, interactive
