class Skeleton:

    def __init__(self, id, style="", value="", relative=None, model=False):
        self.id = id
        self.value = value
        self.style = style
        self.relative = relative
        self.model = model

    def __str__(self):
        return f'Skeleton[id={self.id}]'
