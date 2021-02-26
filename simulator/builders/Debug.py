import esper
import logging
from xml.etree.ElementTree import Element


TYPE = 'debug'


def build_object(cell: Element, world: esper.World, window_options, draw2entity: dict):
    logger = logging.getLogger(__name__)
    logger.debug(f'Ignoring debug entity...')
    return {}, [], {}
