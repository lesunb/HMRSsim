class Position:
    def __init__(self, x=0.0, y=0.0, angle=0.0, w=0.0, h=0.0, movable=True):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.angle = angle
        self.changed = True
        self.movable = movable

    def __str__(self):
        return "Position[({},{}) {} {} {}]".format(self.x, self.y, self.w, self.h, self.angle)