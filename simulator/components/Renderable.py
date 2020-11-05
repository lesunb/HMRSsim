class Renderable:
    def __init__(self, sprite, primitive=False, center=None):
        self.sprite = sprite
        self.w = sprite.width if not primitive else None 
        self.h = sprite.height if not primitive else None
        self.center = center
        self.initialized = False
        self.is_primitive = primitive

    def __str__(self):
        return f'Renderable[primitive={self.is_primitive}]'