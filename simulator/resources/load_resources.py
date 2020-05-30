import pyglet
import os
import xml.etree.ElementTree as ET

from simulator.utils.mxgraph import inflate


working_dir = os.path.dirname(os.path.realpath(__file__))
images_dir = os.path.join(working_dir, 'images')
map_dir = os.path.join(working_dir, 'map')

pyglet.resource.path = [working_dir, images_dir]
pyglet.resource.reindex()


def sprite(batch, image="redsquare.png", x=0, y=0):
    image = pyglet.resource.image(image)
    return pyglet.sprite.Sprite(img=image,
                                x=x,
                                y=y,
                                batch=batch)


def mapFromDrawio(batch, drawioxml):
    tree = ET.parse(map_dir + drawioxml)
    root = tree.getroot()
    for child in root:
        decoded_content = inflate(child.text, b64=True)
        diagram = ET.fromstring(decoded_content)
        print(diagram)
