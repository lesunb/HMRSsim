from main import Simulator
from simulator.components.Position import Position
from simulator.components.CollisionHistory import CollisionHistory
from simulator.components.Map import Map
from typehints.component_types import Component

class TestHelper:
    def __init__(self, simulation: Simulator):
        """Receives a Simulation instance as a parameter."""
        self.simulation = simulation

    def get_object_id(self, drawio_id: str) -> int:
        """Gets the esper entity id from a drawio object id.

        - drawio_id: property id from an object element tree.
        """
        drawio_id = str(drawio_id)

        for obj in self.simulation.objects:
            if obj[1] == drawio_id:
                return obj[0]
        return None

    def get_drawio_id(self, drawio_id: str) -> int:
        """Gets the esper entity id from a drawio 'draw2ent' id.

        - drawio_id: property id of an style element (is not an object) of drawio file (xml).
        """
        entity_id = self.simulation.draw2ent.get(drawio_id, None)
        if entity_id:
            return entity_id[0]
        return entity_id

    def get_pickable_id(self, name: str) -> int:
        """
        Gets the esper entity id from a pickable name.

        - name: property of an element tree of type 'pickable'.
        """
        return self.simulation.interactive.get(name, None)

    def cast_id(self, drawio_id: str) -> int:
        """
        Transforms the id present in the drawio xml to the esper entity id.

        - In case the object is a pickable, pass the name property as the
        parameter to the drawio_id.
        """
        entity_id = self.get_object_id(drawio_id)
        if entity_id:
            return entity_id
        entity_id = self.get_pickable_id(drawio_id)
        if entity_id:
            return entity_id
        return self.get_drawio_id(drawio_id)


    def get_component(self, component: Component, drawio_id: str) -> Component:
        """
        Gets the component from the entity.

        - component: the component class.
        - drawio_id: property id from an element of the drawio file.
        """
        entity_id = self.cast_id(drawio_id)
        if entity_id and self.simulation.world.has_component(entity_id, component):
            return self.simulation.world.component_for_entity(entity_id, component)
        else:
            return None

    def get_position(self, drawio_id: str) -> Position:
        """
        Gets the Position component from an entity.

        - drawio_id: property id from an element of the drawio file.
        """
        return self.get_component(Position, drawio_id)

    def set_position(self, drawio_id: str, x: float, y: float) -> None:
        """
        Changes the position of an entity.

        - drawio_id: property id from an element of the drawio file.
        - x: the new x-axis value.
        - y: the new y-axis value
        """
        position = self.get_component(Position, drawio_id)
        if position:
            position.x = x
            position.y = y
            position.center = (position.x + position.w // 2, position.y + position.h // 2) # update the center position

    def get_center(self, drawio_id: str):
        """
        Returns a tuple with the center position (x,y) of the entity.

        - drawio_id: property id from an element of the drawio file.
        """
        position = self.get_component(Position, drawio_id)
        if position:
            return position.center
        return None

    def get_poi(self, poi_tag: str):
        """Returns a tuple with the position values (x,y) of the POI."""
        world_map = self.simulation.world.component_for_entity(1, Map)
        if world_map:
            return world_map.pois.get(poi_tag, None)
        return None

    def get_collision(self, entity_id: str, other_entity_id: str) -> CollisionHistory:
        """
        Gets the collision event that occurred between the entity and the other_entity.

        - entity_id, other_entity_id: the property id of the element (drawio file).
        """
        collision = self.get_component(CollisionHistory, entity_id)
        other_entity_id = self.cast_id(other_entity_id)

        if collision and other_entity_id in collision.collisions:
            return collision.collisions[other_entity_id]
        return None