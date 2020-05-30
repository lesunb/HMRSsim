class Renderable:
    def __init__(self, sprite):
        self.sprite = sprite
        self.w = sprite.width
        self.h = sprite.height
        self.initialized = False

