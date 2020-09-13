

class Collidable:
    def __init__(self, shape, collision_tag='generic collision'):
        if isinstance(shape, list):
            self.shapes = shape
        else:
            self.shapes = [shape]
        self.event_tag = collision_tag

    def __str__(self):
        return "Collidable Component"
