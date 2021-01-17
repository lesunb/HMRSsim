"""This file can be imported to create models from a XML drawio file.
   TODO: Complete description with exported functions...
"""
from typing import Tuple

import utils.helpers as helpers
from . import Wall, WallCorner, WallU, Room, Shape
from components.Skeleton import Skeleton

# TODO: Make this method extensible with new parser funtions for different shapes
# Hold parse functions on a dict
# def parse_mxCell(el, batch, windowOptions):
def parse_mxCell(el, windowOptions):
    """ Parses an mxCell extracted from .drawio XML (the simulation map)
        TODO: Complete description...
    """
    if el.tag != 'mxCell':
        raise Exception(f"Element {el.tag} is not mxCell.")

    windowSize, lineWidth = windowOptions
    cell_style = helpers.parse_style(el.attrib['style'])
    style = cell_style.get('shape', '')
    obj = None
    if style == 'mxgraph.floorplan.wallCorner':
        obj = WallCorner.from_mxCell(el, windowSize, lineWidth)
    elif style == 'mxgraph.floorplan.wallU':
        obj = WallU.from_mxCell(el, windowSize, lineWidth)
    elif style == 'mxgraph.floorplan.room':
        obj = Room.from_mxCell(el, windowSize, lineWidth)
    elif style == 'mxgraph.floorplan.wall':
        obj = Wall.from_mxCell(el, windowSize, lineWidth)
    else:
        obj = Shape.from_mxCell(el, windowSize, lineWidth)
        obj = obj[0:2]
    # Adds the cell id before returning
    obj[1]['id'] = el.attrib['id']
    pos = obj[0][0]
    obj[0].append(Skeleton(id=el.attrib['id'], style=el.attrib['style']))
    return obj


# def parse_object(el, batch, windowOptions):
def parse_object(el, windowOptions):
    windowSize, lineWidth = windowOptions
    # obj = Shape.from_object(el, batch, windowSize, lineWidth)
    obj = Shape.from_object(el, windowSize, lineWidth)
    obj[1]['id'] = el.attrib['id']
    obj[0].append(Skeleton(id=el.attrib['id'], style=el[0].attrib['style'], value=el.attrib.get('label', '')))
    return obj
