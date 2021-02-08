from typehints.component_types import Component


class Skeleton(Component):

    def __init__(self, id: str, style="", value="", relative=None, model=False):
        self.id = id
        self.value = value
        self.style = style
        self.relative = relative
        self.model = model

    def __str__(self):
        return f'Skeleton[id={self.id}]'
