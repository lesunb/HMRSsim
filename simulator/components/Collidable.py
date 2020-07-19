

class Collidable:
    def __init__(self, shape):
        if isinstance(shape, list):
            self.shapes = shape
        else:
            self.shapes = [shape]

    def __str__(self):
        return "Collidable Component"
