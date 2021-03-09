from components.Position import Position
from components.Collision import Collision
from components.Map import Map

class TestHelper:
    def __init__(self, simulation):
        self.simulation = simulation

    def get_object_id(self, drawio_id):
        """Gets the simulation entity id given the id of the drawio object."""
        drawio_id = str(drawio_id)

        for obj in self.simulation.objects:
            if obj[1] == drawio_id:
                return obj[0]
        return None

    def get_drawio_id(self, drawio_id):
        entity_id = self.simulation.draw2ent.get(drawio_id, None)  
        if entity_id:
            return entity_id[0]
        return entity_id

    def get_pickable_id(self, name):
        return self.simulation.interactive.get(name, None)

    def cast_id(self, drawio_id):
        """Converts the object id present in the xml tree to the object id in the simulation.
        Tipos possíveis:
        - No caso do objeto ser do tipo pickable, passar a propriedade name como
        parametro para o drawio_id
        """
        entity_id = self.get_object_id(drawio_id)
        if entity_id:
            return entity_id
        entity_id = self.get_pickable_id(drawio_id) 
        if entity_id:
            return entity_id   
        return self.get_drawio_id(drawio_id)


    def get_component(self, component, drawio_id):
        entity_id = self.cast_id(drawio_id)
        if entity_id and self.simulation.world.has_component(entity_id, component):
            return self.simulation.world.component_for_entity(entity_id, component)
        else:
            return None
    
    def get_position(self, drawio_id):
        return self.get_component(Position, drawio_id)
    
    def set_position(self, drawio_id, x, y):
        position = self.get_component(Position, drawio_id)
        if position:
            position.x = x
            position.y = y

    def get_center(self, drawio_id):
        position = self.get_component(Position, drawio_id)
        if position:
            return position.center
        return None
    
    def get_poi(self, poi_tag):
        map = self.simulation.world.component_for_entity(1, Map)
        return map.pois[poi_tag]

    def get_collision(self, entity_id, other_entity_id):
        collision = self.get_component(Collision, entity_id) 
        other_entity_id = self.cast_id(other_entity_id)
        
        if collision and other_entity_id in collision.collisions:  
            return collision.collisions[other_entity_id]
        return None