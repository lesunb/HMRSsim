import pyglet


class Label:

    def __init__(self, label, pos, batch):
        self.labelTag = pyglet.text.HTMLLabel(label,
                                              batch=batch,
                                              x=pos[0], y=pos[1],
                                              anchor_x='center', anchor_y='center')
