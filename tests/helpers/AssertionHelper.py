from simulator.components.Collision import Collision
from tests.helpers.TestHelper import TestHelper

class AssertionHelper(TestHelper):
    def __init__(self, simulation) -> None:
        """Receives a Simulation instance as a parameter."""
        super().__init__(simulation)

    def have_collided(self, entity_id: str, other_entity_id: str):
        """Assert that a collision has occurred between the entity and the other_entity."""
        collision = self.get_component(Collision, entity_id) # se for None, ou não existe Collision ou não existe entity_id
        other_id = self.cast_id(other_entity_id)

        # TODO: verificar se exite a entity_id
        if not other_id:
            raise AssertionError(f'There is no entity named {other_entity_id}')

        if collision and other_id in collision.collisions:
            return True
        raise AssertionError(f'There is no collision between {entity_id} and {other_entity_id}')

    def is_in_poi(self, drawio_id: str, poi_tag: str):
        """Assert if the center of the object is positioned at the poi (point of interest).

        - drawio_id: property id from an element of the drawio file.
        - poi_tag: the poi_tag property of a POI
        """
        entity_center = self.get_center(drawio_id)
        if not entity_center:
            raise AssertionError(f'There is no entity named {drawio_id}')

        poi = self.get_poi(poi_tag)
        if not poi:
            raise AssertionError(f'There is no POI named {poi_tag}')

        if (entity_center[0] == poi[0]) and (entity_center[1] == poi[1]):
            return True
        raise AssertionError(f'The center of {drawio_id} {entity_center} is not positioned at the POI {poi_tag} {poi}.')

    def robot_drop_pickable_in_poi(self, robot_id: str, pickable_name: str, poi_tag: str):
        """Assert if the pickable is in the poi that the robot has left.

        - robot_id: drawio id of the robot that held the pickable.
        - pickable_name: the name property of the grabbed object, typed 'pickable'.
        - poi_tag: the tag property of the destination poi.
        """
        pickable_position = self.get_center(pickable_name)
        if not pickable_name:
            raise AssertionError(f'There is no pickable named {pickable_name}')

        poi = self.get_poi(poi_tag)
        if not poi:
            raise AssertionError(f'There is no POI named {poi_tag}')

        robot_height = self.get_position(robot_id)
        if not robot_height:
            raise AssertionError(f'There is no entity named {robot_id}')

        robot_height = robot_height.h

        if (pickable_position[0] == poi[0]) and (pickable_position[1] == poi[1] + robot_height):
            return True

        raise AssertionError(f'The pickable is not in poi {poi_tag}.\nPickable actual center position: {pickable_position}\nPoi position (x, y + robot height): ({poi[0]}, {poi[1] + robot_height})')

    def is_in_center_of(self, entity_id, other_entity_id):
        """
        Asserts that the center of entity is the same center of the other_entity.

        - entity_id, other_entity_id: the property id of the element (drawio file).
        """
        entity_center = self.get_center(entity_id)
        if not entity_center:
            raise AssertionError(f'There is no entity named {entity_id}')
        
        other_entity_center = self.get_center(other_entity_id)
        if not other_entity_center:
            raise AssertionError(f'There is no entity named {other_entity_id}')

        if entity_center[0] == other_entity_center[0] and entity_center[1] == other_entity_center[1]:
            return True
        raise AssertionError((f'The center of {entity_id} is not the same center of the {other_entity_id}.\n'
                            f'{entity_id} center position: {entity_center}\n'
                            f'{other_entity_id} center position: {other_entity_center}'))

    def is_in_same_position(self, entity_id, other_entity_id):
        """
        Asserts that the (x,y) position of entity is the same of the other_entity.

        - entity_id, other_entity_id: the property id of the element (drawio file).
        """
        entity_position = self.get_position(entity_id)
        if not entity_position:
            raise AssertionError(f'There is no entity named {entity_id}')
    
        other_entity_position = self.get_position(other_entity_id)
        if not other_entity_position:
            raise AssertionError(f'There is no entity named {other_entity_id}')

        if entity_position.x == other_entity_position.x and entity_position.y == other_entity_position.y:
            return True
        raise AssertionError((f'The position of {entity_id} is not the same of the {other_entity_id}.\n'
                              f'The {entity_id} position: {entity_position.x, entity_position.y}\n'
                              f'{other_entity_id} position: {other_entity_position.x, other_entity_position.y}'))

    
    def is_in_position(self, entity_id, position):
        """
        Asserts that the entity is int the (x, y) position passed as parameter.

        - entity_id: the property id of the element (drawio file).
        - position: a coordinate (x, y) of the map.
        """
        entity_position = self.get_position(entity_id)
        if not entity_position:
            raise AssertionError(f'There is no entity named {entity_id}')

        if entity_position.x == position[0] and entity_position.y == position[1]:
            return True
        raise AssertionError((f'The entity {entity_id} is not at the expected position.\n'
                             f'Actual entity position: {entity_position.x, entity_position.y}.\n'
                             f'Expected entity position: ({position[0]}, {position[1]}).'))
