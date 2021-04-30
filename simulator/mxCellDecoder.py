"""This file can be imported to create models from a XML drawio file.
   TODO: Complete description with exported functions...
"""
import typing
from simulator import dynamic_models
import utils.helpers as helpers

from simulator.components.Skeleton import Skeleton
from simulator.typehints.build_types import WindowOptions
from xml.etree.ElementTree import Element

available_models = dynamic_models.export_available_models()


def parse_mxCell(el: Element, window_options: WindowOptions):
    """Parses an mxCell extracted from .drawio XML"""
    if el.tag != 'mxCell':
        raise Exception(f"Element {el.tag} is not mxCell.")

    window_size, line_width = window_options
    cell_style = helpers.parse_style(el.attrib['style'])
    style = cell_style.get('shape', '')
    if style in available_models:
        obj = available_models[style].__dict__['from_mxCell'](el, line_width)
    else:
        obj = available_models['default'].__dict__['from_mxCell'](el, line_width)
    # Adds the cell id before returning
    obj[1]['id'] = el.attrib['id']
    pos = obj[0][0]
    obj[0].append(Skeleton(id=el.attrib['id'], style=el.attrib['style']))
    return obj


# def parse_object(el, batch, windowOptions):
def parse_object(el, window_options, shape='default'):
    windowSize, lineWidth = window_options
    obj = available_models[shape].__dict__['from_object'](el, lineWidth)
    obj[1]['id'] = el.attrib['id']
    obj[0].append(Skeleton(id=el.attrib['id'], style=el[0].attrib['style'], value=el.attrib.get('label', '')))
    return obj
