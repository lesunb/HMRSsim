import esper

from components.Renderable import Renderable
from components.Position import Position


class RenderProcessor(esper.Processor):
    def __init__(self):
        super().__init__()

    def process(self, dt):
        # This will iterate over every Entity that has BOTH of these components:
        for ent, (pos, rend) in self.world.get_components(Position, Renderable):
            if not pos.changed:
                continue
            # Update the Renderable Component's position by it's Velocity:
            # An example of keeping the sprite inside screen boundaries. Basically,
            # adjust the position back inside screen boundaries if it is outside:
            rend.sprite.position = (pos.x, pos.y)
