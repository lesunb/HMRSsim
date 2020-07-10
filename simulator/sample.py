import pyglet
import esper
import os
import sys

# Include current directory in path
sys.path.append(os.getcwd())

from primitives import Ellipse

window = pyglet.window.Window(width=500,
                              height=500,
                              caption="Sample")
batch = pyglet.graphics.Batch()

el = Ellipse((100,100), 100, 50)
el.add_to_batch(batch)


@window.event
def on_draw():
    # Clear the window to background color
    window.clear()
    # Draw the batch of Renderables:
    batch.draw()

if __name__ == "__main__":
    # NOTE!  schedule_interval will automatically pass a "delta time" argument
    #        to world.process, so you must make sure that your Processor classes
    #        account for this. See the example Processors above.
    pyglet.app.run()