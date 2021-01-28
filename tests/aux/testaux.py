class TestApi():
    def __init__(self, world):
        self.world = world

    def find_component(self, component, entity_id):
        component = self.world.component_for_entity(entity_id, component)
        return component
