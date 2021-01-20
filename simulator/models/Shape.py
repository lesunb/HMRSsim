import primitives as primitives
import json
from collision import Poly, Vector
from components.Collidable import Collidable
from components.Position import Position
from components.Velocity import Velocity
from components.POI import POI
from components.BatteryComponent import Battery
from utils.helpers import parse_style, translate_coordinates

MODEL = 'default'
def from_object(el, windowSize, lineWidth=10):
    options = el.attrib

    components, style = from_mxCell(el[0], windowSize, lineWidth)
    if 'collidable' not in options:
        options['collidable'] = True
    if 'movable' not in options:
        options['movable'] = True
    if 'POI' in options:
        points = json.loads(options['POI'])
        points = list(map((lambda p: translate_coordinates(p, windowSize, 0)), points))
        components.append(POI(points=points))
    if 'battery' in options:
        bat_info = json.loads(options['battery'])
        bat = Battery(charge=float(bat_info['initialCharge']), lookupTable=bat_info['lookupTable'])
        components.append(bat)

    options.update(style)
    pos = components[0]
    if options['type'] == 'robot':
        components.append(Velocity(x=0, y=0))
    if 'collision_tag' in options:
        coll = list(filter(lambda c: isinstance(c, Collidable), components))[0]
        coll.event_tag = options['collision_tag']
    return components, options


def from_mxCell(el, batch, windowSize, lineWidth=10):
    # Parse style
    style = parse_style(el.attrib['style'])
    # Get parent
    style['parent'] = el.attrib['parent']

    # Get geometry
    geometry = el[0]
    x = float(geometry.attrib.get('x', '0'))
    y = float(geometry.attrib.get('y', '0'))
    width = float(geometry.attrib['width'])
    height = float(geometry.attrib['height'])
    # Create drawing
    (x, y) = translate_coordinates((x, y), windowSize, height)
    pos = Position(x=x, y=y, w=width, h=height, movable=False)

    rotate = 0
    if style.get('rotation', '') != '':
        rotate = int(style['rotation'])
        if rotate < 0:
            rotate = 360 + rotate
    pos.angle = rotate
    center = (pos.x + pos.w // 2, pos.y + pos.h // 2)

    if 'ellipse' in style:
        draw = primitives.Ellipse(center, width, height, style, rotate)
        col_points = draw._get_points()
    else:
        col_points = pos._get_box()

    col_points = list(map(lambda x: Vector(x[0] - center[0], x[1] - center[1]), col_points))
    collision_box = Poly(Vector(center[0], center[1]), col_points)

    return [pos, Collidable(shape=collision_box)], style
