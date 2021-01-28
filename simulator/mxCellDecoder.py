"""This file can be imported to create models from a XML drawio file.
   TODO: Complete description with exported functions...
"""
from typing import Tuple

import utils.helpers as helpers
from components.Skeleton import Skeleton
import dynamic_models

available_models = dynamic_models.export_available_models()


def parse_mxCell(el, windowOptions):
    """ Parses an mxCell extracted from .drawio XML (the simulation map)
        TODO: Complete description...
    """
    if el.tag != 'mxCell':
        raise Exception(f"Element {el.tag} is not mxCell.")

    windowSize, lineWidth = windowOptions
    cell_style = helpers.parse_style(el.attrib['style'])
    style = cell_style.get('shape', '')
    if style in available_models:
        obj = available_models[style].__dict__['from_mxCell'](el, windowSize, lineWidth)
    else:
        obj = available_models['default'].__dict__['from_mxCell'](el, windowSize, lineWidth)
    # Adds the cell id before returning
    obj[1]['id'] = el.attrib['id']
    pos = obj[0][0]
    obj[0].append(Skeleton(id=el.attrib['id'], style=el.attrib['style']))
    return obj


# def parse_object(el, batch, windowOptions):
def parse_object(el, windowOptions, shape='default'):
    windowSize, lineWidth = windowOptions
    obj = available_models[shape].__dict__['from_object'](el, windowSize, lineWidth)
    obj[1]['id'] = el.attrib['id']
    obj[0].append(Skeleton(id=el.attrib['id'], style=el[0].attrib['style'], value=el.attrib.get('label', '')))
    return obj
