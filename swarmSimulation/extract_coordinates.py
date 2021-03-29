
from simulator.resources import load_resources as ld

name, content = ld.map_from_drawio('swarmSimulation/shapes.drawio')
root = content[0]
drone_cells = list(map(lambda x: x[0], filter(lambda obj: obj.tag == 'object', root)))
SHAPE = []
for d in drone_cells:
    geo = d[0]
    x = geo.attrib['x']
    y = geo.attrib['y']
    SHAPE.append((float(x), float(y)))

SHAPE.sort(key=lambda obj: obj[1])
name = input('Shape name:')
print(f'"{name}": {SHAPE}')
