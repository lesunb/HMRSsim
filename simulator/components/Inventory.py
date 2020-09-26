
class Inventory:

    def __init__(self, objects=None):
        if objects is None:
            objects = {}
        self.objects = objects
