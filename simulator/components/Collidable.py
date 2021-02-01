

class Collidable:
    def __init__(self, shape, collision_tag='genericCollision'):
        if isinstance(shape, list):
            self.shapes = shape
        else:
            self.shapes = [shape]
        self.event_tag = collision_tag

    def __str__(self):
        return f"Collidable[{len(self.shapes)} shapes. Tag={self.event_tag}]"
