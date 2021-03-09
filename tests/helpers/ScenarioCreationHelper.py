from systems.GotoDESProcessor import GotoPoiEventTag, GotoPoiPayload, GotoPosEventTag, GotoPosPayload
from systems.PathProcessor import PathProcessor
from systems.MovementProcessor import MovementProcessor
import systems.GotoDESProcessor as NavigationSystem
import systems.ClawDESProcessor as ClawProcessor
import systems.ScriptEventsDES as ScriptSystem
import systems.CollisionDetectorDESProcessor as collisionDetector
from systems.CollisionProcessor import CollisionProcessor
from components.Path import Path
from components.Map import Map
from typehints.component_types import EVENT
from tests.helpers.TestHelper import TestHelper

class ScenarioCreationHelper(TestHelper):
    def __init__(self, simulation):
        super().__init__(simulation)
    
    def add_component(self, component, drawio_id):
        entity_id = self.cast_id(drawio_id)
        self.simulation.world.add_component(entity_id, component)
    
    def create_path(self, robot_id, points):
        """points: List of dict."""
        path = self.get_component(Path, robot_id)
        if path is None:
            path = Path(points)
            self.add_component(path, robot_id)
        else:
            for point in points:
                path.points.append(point)

    def add_goto_position_event(self, drawio_id, position):
        entity_id = self.cast_id(drawio_id)
        x = position[0]
        y = position[1]
        payload = GotoPosPayload(entity_id, [x, y])
        new_event = EVENT(GotoPosEventTag, payload)
        event_store = self.simulation.KWARGS['EVENT_STORE']
        event_store.put(new_event)

    def add_goto_poi_event(self, drawio_id, poi_tag):
        entity_id = self.cast_id(drawio_id)
        payload = GotoPoiPayload(entity_id, poi_tag)
        new_event = EVENT(GotoPoiEventTag, payload)
        event_store = self.simulation.KWARGS['EVENT_STORE']
        event_store.put(new_event)

    def add_ability_to_follow_path(self):
        width, height = self.simulation.window_dimensions
        self.simulation.add_system(PathProcessor())
        self.simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

    def add_ability_to_navigate(self):
        NavigationSystemProcess = NavigationSystem.init()
        self.simulation.add_des_system((NavigationSystemProcess,))
        width, height = self.simulation.window_dimensions
        self.simulation.add_system(PathProcessor())
        self.simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

    def add_ability_to_collide(self):
        self.simulation.add_system(CollisionProcessor())
        self.simulation.add_des_system((collisionDetector.process,))

    def add_ability_to_move_to_specific_poi(self):
        NavigationSystemProcess = NavigationSystem.init()
        self.simulation.add_des_system((NavigationSystemProcess,))
        width, height = self.simulation.window_dimensions
        self.simulation.add_system(PathProcessor())
        self.simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))
    
    def add_ability_to_move_to_specific_position(self):
        NavigationSystemProcess = NavigationSystem.init()
        self.simulation.add_des_system((NavigationSystemProcess,))
        width, height = self.simulation.window_dimensions
        self.simulation.add_system(PathProcessor())
        self.simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

    def add_script_ability(self):
        extra_instructions = [
            (NavigationSystem.GotoInstructionId, NavigationSystem.goInstruction),
            (ClawProcessor.GrabInstructionTag, ClawProcessor.grabInstruction),
            (ClawProcessor.DropInstructionTag, ClawProcessor.dropInstrution)
        ]
        ScriptProcessor = ScriptSystem.init(extra_instructions, [ClawProcessor.ClawDoneTag])
        self.simulation.add_des_system((ScriptProcessor,),)
        self.simulation.add_des_system((ClawProcessor.process,))

    def add_poi(self, poi_tag, poi_value):
        map = self.simulation.world.component_for_entity(1, Map)
        map.pois[poi_tag] = poi_value

    # add seer
    # add log json