class Position:
    def __init__(self, x=0.0, y=0.0, angle=0.0, w=0.0, h=0.0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.angle = angle
        self.changed = True
