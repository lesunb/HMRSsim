from typing import List

import pyglet
import esper
import copy
import json

import dynamic_importer

import utils.helpers as helpers
import resources.load_resources as loader
import models.mxCellDecoder as mxCellDecoder

from components.Path import Path
from components.Map import Map
from components.Inventory import Inventory
from components.Pickable import Pickable
from components.Skeleton import Skeleton


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
    window = pyglet.window.Window(width=width,
                                  height=height,
                                  caption=window_name)
    batch = pyglet.graphics.Batch()
    # Define default line width
    pyglet.gl.glLineWidth(line_width)
    # Define background clear color
    pyglet.gl.glClearColor(BKGD[0], BKGD[1], BKGD[2], BKGD[3])
    pyglet.gl.glClear(pyglet.gl.GL_COLOR_BUFFER_BIT | pyglet.gl.GL_DEPTH_BUFFER_BIT)

    world = esper.World()
    simulation = world.create_entity()  # Simulation is always the first entity
    draw_map, objects, interactive = build_simulation_objects(content_root, batch, world, ((width, height), line_width))
    world.add_component(simulation, Inventory(interactive))
    skeleton_style = "{{\"width\":{:d},\"height\":{:d}}}".format(width, height)
    world.add_component(simulation, Skeleton(id=window_name, style=skeleton_style, model=True))
    return {
        'world': world,
        'window': window,
        'batch': batch,
        'window_props': (window_name, (width, height), BKGD),
        'draw_map': draw_map,
        'objects': objects
    }


def build_simulation_objects(content_root, batch: pyglet.graphics.Batch, world: esper.World, window_options):
    # Create Walls
    draw2entity = {}
    interactive = {}
    objects = []
    windowSize = window_options[0]
    for cell in content_root:
        if cell.tag == 'mxCell' and 'style' in cell.attrib:
            (components, style) = mxCellDecoder.parse_mxCell(cell, batch, window_options)
            ent = world.create_entity()
            for c in components:
                world.add_component(ent, c)
            draw2entity[style['id']] = [ent, style]
        if cell.tag == 'object':
            if cell.attrib['type'] == 'robot':
                (components, style) = mxCellDecoder.parse_object(cell, batch, window_options)
                ent = world.create_entity()
                # Custom components
                for key, val in cell.attrib.items():
                    if key.startswith('component_'):
                        component_name = key[10:]  # removes "component_" from the name
                        init_values = json.loads(val)
                        component = dynamic_importer.init_component(component_name, init_values)
                        components.append(component)
                for c in components:
                    world.add_component(ent, c)
                draw2entity[style['id']] = [ent, style]
                objects.append((ent, style['id']))
            elif cell.attrib['type'] == 'pickable':
                skeleton = copy.copy(cell)
                (components, style) = mxCellDecoder.parse_object(cell, batch, window_options)
                pick = Pickable(float(cell.attrib['weight']), cell.attrib['name'], skeleton)
                components.append(pick)
                ent = world.create_entity()
                for c in components:
                    world.add_component(ent, c)
                interactive[style['name']] = ent
            elif cell.attrib['type'] == 'path':
                mxCell = cell[0]
                points = Path.from_mxCell(mxCell, windowSize[1])
                obj = mxCell.attrib.get('source', None)
                (ent, _) = draw2entity.get(obj, (None, None))
                if ent is None:
                    print(f"Path origin ({obj}) not found. Trying target.")
                else:
                    print(f"Adding path to entity {ent}")
                    world.add_component(ent, Path(points))
            elif cell.attrib['type'] == 'map-path':
                mxCell = cell[0]
                points = Path.from_mxCell(mxCell, windowSize[1])
                objId = cell.attrib.get('origin', '')
                key = cell.attrib.get('key', '')
                if key == '':
                    print(f"Map entry without key. Using default value")
                    key = 'Default'
                (ent, _) = draw2entity.get(objId, (None, None))
                if ent is None:
                    print(f"Path origin ({obj}) not found. Trying target.")
                else:
                    if world.has_component(ent, Map):
                        map = world.component_for_entity(ent, Map)
                        if key == 'Default':
                            key += str(len(map))
                        map.paths[key] = points
                    else:
                        if key == 'Default':
                            key += '0'
                        newMap = Map({key: points})
                        world.add_component(ent, newMap)
            else:
                print("Unrecognized object", cell)
    return draw2entity, objects, interactive
