import esper
import copy
import json

import dynamic_importer

import utils.helpers as helpers
import resources.load_resources as loader
import mxCellDecoder as mxCellDecoder

from dynamic_builders import export_available_builders
from components.Inventory import Inventory
from components.Skeleton import Skeleton

available_builders = export_available_builders()


def build_simulation_from_map(file, line_width=10):
    # Load map from .drawio file
    window_name, map_content = loader.mapFromDrawio(file)
    width = int(map_content.attrib.get('pageWidth', 500))
    height = int(map_content.attrib.get('pageHeight', 500))

    BKGD = helpers.hex_to_rgb('#FFFFFF')
    if 'background' in map_content.attrib:
        BKGD = helpers.hex_to_rgb(map_content.attrib['background'])
    content_root = map_content[0]
    # Create pyglet window
    # window = pyglet.window.Window(width=width,
    #                               height=height,
    #                               caption=window_name)
    # batch = pyglet.graphics.Batch()
    # Define default line width
    # pyglet.gl.glLineWidth(line_width)
    # Define background clear color
    # pyglet.gl.glClearColor(BKGD[0], BKGD[1], BKGD[2], BKGD[3])
    # pyglet.gl.glClear(pyglet.gl.GL_COLOR_BUFFER_BIT | pyglet.gl.GL_DEPTH_BUFFER_BIT)

    world = esper.World()
    simulation = world.create_entity()  # Simulation is always the first entity
    draw_map, objects, interactive = build_simulation_objects(content_root, world, ((width, height), line_width))
    world.add_component(simulation, Inventory(interactive))
    skeleton_style = "{{\"width\":{:d},\"height\":{:d}}}".format(width, height)
    world.add_component(simulation, Skeleton(id=window_name, style=skeleton_style, model=True))
    return {
        'world': world,
        'window_props': (window_name, (width, height), BKGD),
        'draw_map': draw_map,
        'objects': objects
    }


def build_simulation_objects(content_root, world: esper.World, window_options):
    # Create Walls
    draw2entity = {}
    objects = []
    interactive = {}
    for cell in content_root:
        if cell.tag == 'mxCell' and 'style' in cell.attrib:
            (components, style) = mxCellDecoder.parse_mxCell(cell, window_options)
            ent = world.create_entity()
            for c in components:
                world.add_component(ent, c)
            draw2entity[style['id']] = [ent, style]
        if cell.tag == 'object':
            type = cell.attrib['type']
            pendingUpdates = available_builders[type].__dict__['build_object'](cell, world, window_options, draw2entity)
            draw2entity.update(pendingUpdates[0])
            objects += pendingUpdates[1]
            interactive.update(pendingUpdates[2])

    return draw2entity, objects, interactive
