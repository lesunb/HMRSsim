
import pyglet
import esper
import os
import sys

# Include current directory in path
sys.path.append(os.getcwd())

import simulator.utils.helpers as helpers
import simulator.resources.load_resources as loader
import simulator.models.mxCellDecoder as mxCellDecoder

from components.Path import Path

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
  (draw_map, objects) = build_simulation_objects(content_root, batch, world, ((width, height), line_width))
  return {
      'world': world,
      'window': window,
      'batch': batch,
      'window_props': (window_name, (width, height), BKGD),
      'draw_map': draw_map,
      'objects': objects,
  }

def build_simulation_objects(content_root, batch: pyglet.graphics.Batch, world: esper.World, window_options):
    # Create Walls
    draw2entity = {}
    objects = []
    windowSize = window_options[0]
    for cell in content_root:
        if cell.tag == 'mxCell' and 'style' in cell.attrib:
            (components, style) = mxCellDecoder.parse_mxCell(cell, batch, window_options)
            ent = world.create_entity()
            for c in components:
                world.add_component(ent, c)
            draw2entity[style['id']] = (ent, style)
        if cell.tag == 'object':
            if cell.attrib['type'] == 'robot':
                (components, style) = mxCellDecoder.parse_object(cell, batch, window_options)
                ent = world.create_entity()
                for c in components:
                    world.add_component(ent, c)
                draw2entity[style['id']] = (ent, style)
                objects.append(ent)
            elif cell.attrib['type'] == 'path':
                mxCell = cell[0]
                points = Path.from_mxCell(mxCell, windowSize[1])
                obj = mxCell.attrib.get('source', None)
                (ent, _) = draw2entity.get(obj, None)
                if ent is None:
                    print(f"Path origin ({obj}) not found")
                else:
                    print(f"Adding path to entity {ent}")
                    world.add_component(ent, Path(points))
            else:
                print("Unrecognized object", cell)
    return (draw2entity, objects)