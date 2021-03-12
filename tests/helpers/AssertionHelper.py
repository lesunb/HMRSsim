from simulator.components.Collision import Collision
from tests.helpers.TestHelper import TestHelper

class AssertionHelper(TestHelper):
    def __init__(self, simulation) -> None:
        super().__init__(simulation)

    def have_collided(self, entity_id, other_entity_id):
        """Verifica se ocorreu 1 colisao"""
        collision = self.get_component(Collision, entity_id) 
        other_entity_id = self.cast_id(other_entity_id)
        
        if collision and other_entity_id in collision.collisions: 
            return True
        return False

    def is_in_poi(self, entity_id, poi_tag):
        entity_position = self.get_position(entity_id)  # TODO: verificar se é para pegar o centro
        poi = self.get_poi(poi_tag)

        if entity_position.x == poi[0] and entity_position.y == poi[1]:
            return True
        else:
            return False
            # TODO: raise AssertionError(f'Entity is not in the POI. Entity Position: {entity_position}. Poi: {poi}')
            # como retornar o Assertion e o False

    def is_in_center_of(self, entity_id, other_entity_id):
        """Compara o x e o y do centro"""
        entity_center = self.get_center(entity_id)
        other_entity_center = self.get_center(other_entity_id)

        if entity_center[0] == other_entity_center[0] and entity_center[1] == other_entity_center[1]:
            return True
        return False
    
    def is_in_same_position(self, entity_id, other_entity_id):
        """Compara o x e o y."""
        entity_position = self.get_position(entity_id)
        other_entity_position = self.get_position(other_entity_id)

        if entity_position.x == other_entity_position.x and entity_position.y == other_entity_position.y:
            return True
        return False
    
    def is_in_position(self, entity_id, position):
        # TODO: melhorar nome dos metodos
        entity_position = self.get_position(entity_id)
        if entity_position.x == position[0] and entity_position.y == position[1]:
            return True
        return False
