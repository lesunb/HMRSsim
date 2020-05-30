import pyglet
import os

working_dir = os.path.dirname(os.path.realpath(__file__))
images_dir = os.path.join(working_dir, 'images')
pyglet.resource.path = [working_dir, images_dir]
pyglet.resource.reindex()


def sprite(batch, image="redsquare.png", x=0, y=0):
    image = pyglet.resource.image(image)
    return pyglet.sprite.Sprite(img=image,
                                x=x,
                                y=y,
                                batch=batch)
