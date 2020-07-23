"""This file can be imported to create models from a XML drawio file.
   TODO: Complete description with exported functions...
"""
from typing import Tuple

import simulator.utils.helpers as helpers
from . import Wall, WallCorner, WallU, Room, Shape

def parse_mxCell(el, windowSize: Tuple[int, int], lineWidth=10):
    """ Parses an mxCell extracted from .drawio XML (the simulation map)
        TODO: Complete description...
    """
    if el.tag != 'mxCell':
        raise Error(f"Element {el.tag} is not mxCell.")

    cell_style = helpers.parse_style(el.attrib['style'])
    style = cell_style.get('shape', '')
    if style == 'mxgraph.floorplan.wallCorner':
        return WallCorner.WallCorner.from_mxCell(el, windowSize, lineWidth)
    elif style == 'mxgraph.floorplan.wallU':
        return WallU.WallU.from_mxCell(el, windowSize, lineWidth)
    elif style == 'mxgraph.floorplan.room':
        return Room.Room.from_mxCell(el, windowSize, lineWidth)
    elif style == 'mxgraph.floorplan.wall':
        return Wall.Wall.from_mxCell(el, windowSize, lineWidth)
    else:
        return Shape.Shape.from_mxCell(el, windowSize, lineWidth)


