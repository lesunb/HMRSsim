import pyglet
import os
import sys
import zlib
import base64

from urllib.parse import unquote
import xml.etree.ElementTree as ET

working_dir = os.path.dirname(os.path.realpath(__file__))
images_dir = os.path.join(working_dir, 'images')

pyglet.resource.path = [working_dir, images_dir]
pyglet.resource.reindex()

def inflate(b, b64=False):
    """~2016 draw.io started compressing 'using standard deflate'
        https://about.draw.io/extracting-the-xml-from-mxfiles/
        experience has shown this is deflate WITH NO HEADER
    """
    if b64:  # optional, additionally base64 decode
        b = base64.b64decode(b)
    return unquote(zlib.decompress(b, -15).decode('utf8'))

def sprite(batch, image="redsquare.png", x=0, y=0):
    image = pyglet.resource.image(image)
    return pyglet.sprite.Sprite(img=image,
                                x=x,
                                y=y,
                                batch=batch)

def mapFromDrawio(drawioxml):
    tree = ET.parse(drawioxml)
    root = tree.getroot()

    # os elementos estarao compactados no text e a lista nao tera nenhum Element
    if len(list(root[0])) == 0: 
        content = ET.fromstring(inflate(root[0].text, True))
    else:
        content = root[0][0] # pegar o mxGraphModel

    return (root[0].attrib.get('name', "Window 1"), content)
