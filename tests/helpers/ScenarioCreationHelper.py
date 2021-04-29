from typing import List
from simulator.components.Camera import Camera
from simulator.components.Claw import Claw
from simulator.components.Script import Script
from simulator.components.CollisionHistory import CollisionHistory
from simulator.systems.GotoDESProcessor import GotoPoiEventTag, GotoPoiPayload, GotoPosEventTag, GotoPosPayload
from simulator.systems.PathProcessor import PathProcessor
from simulator.systems.MovementProcessor import MovementProcessor
import simulator.systems.GotoDESProcessor as NavigationSystem
import simulator.systems.ClawDESProcessor as ClawProcessor
import simulator.systems.ScriptEventsDES as ScriptSystem
import simulator.systems.StopCollisionDESProcessor as stopCollision
from simulator.systems.CameraProcessor import process_camera_event
import simulator.systems.SensorSystem as SensorSystem
from simpy import Store
import simulator.systems.ApproximationDESProcessor as ApproximationProcessor
import simulator.systems.ManageObjects as ObjectManager
from simulator.systems.CollisionProcessor import CollisionProcessor
import simulator.systems.SeerPlugin as Seer
from simulator.components.Path import Path
from simulator.components.Map import Map
from typehints.component_types import EVENT
from tests.helpers.TestHelper import TestHelper

class ScenarioCreationHelper(TestHelper):
    def __init__(self, simulation):
        """Receives a Simulation instance as a parameter."""
        super().__init__(simulation)

    def add_component(self, component, drawio_id):
        """
        Adds a component to the specified entity (drawio_id).

        - component: instance of a Component.
        - drawio_id: property id from an element of the drawio file.
        """
        entity_id = self.cast_id(drawio_id)
        self.simulation.world.add_component(entity_id, component)

    def create_path(self, robot_id, points):
        """
        Adds a path to the robot going through the specified points.

        - robot_id: the robot property id from the drawio file.
        - points: List of (x,y) positions.
        """
        path = self.get_component(Path, robot_id)
        if path is None:
            path = Path(points)
            self.add_component(path, robot_id)
        else:
            for point in points:
                path.points.append(point)

    def add_goto_position_event(self, drawio_id, position):
        """
        Adds an event for the robot to go to a position passed as a parameter.

        - drawio_id: property id from an element of the drawio file.
        - position: a tuple (x, y) or array [x, y] with x-axis value and y-axis value.
        """
        entity_id = self.cast_id(drawio_id)
        x = position[0]
        y = position[1]
        payload = GotoPosPayload(entity_id, [x, y])
        new_event = EVENT(GotoPosEventTag, payload)
        event_store = self.simulation.KWARGS['EVENT_STORE']
        event_store.put(new_event)

    def add_goto_poi_event(self, drawio_id, poi_tag):
        """
        Adds an event for the robot to go to a POI passed as a parameter.

        - drawio_id: property id from an element of the drawio file.
        - poi_tag: the poi_tag property of a POI.
        """
        entity_id = self.cast_id(drawio_id)
        payload = GotoPoiPayload(entity_id, poi_tag)
        new_event = EVENT(GotoPoiEventTag, payload)
        event_store = self.simulation.KWARGS['EVENT_STORE']
        event_store.put(new_event)

    def add_ability_to_follow_path(self):
        """Adds the systems responsible for guiding the robot through a drawio arrow."""
        width, height = self.simulation.window_dimensions
        self.simulation.add_system(PathProcessor())
        self.simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

    def add_ability_to_navigate(self):
        """Adds the systems responsible for Go to a specific POI or Position."""
        NavigationSystemProcess = NavigationSystem.init()
        self.simulation.add_des_system((NavigationSystemProcess,))
        width, height = self.simulation.window_dimensions
        self.simulation.add_system(PathProcessor())
        self.simulation.add_system(MovementProcessor(minx=0, miny=0, maxx=width, maxy=height))

    def add_ability_to_collide(self):
        """Adds the systems responsible for collisions and for checking if collisions have occurred."""
        self.simulation.add_system(CollisionProcessor())
        self.simulation.add_des_system((stopCollision.process,))

    def add_collision_component(self, drawio_id):
        self.add_component(CollisionHistory(), drawio_id)

    def add_claw_ability(self, drawio_id):
        """Adds the component responsible for grab pickables."""
        claw = self.get_component(Claw, drawio_id)
        if claw is None:
            self.add_component(Claw(80, 1), drawio_id)

    def add_script_ability(self):
        """Adds the systems responsible for make script commands Go, Grab and Drop."""
        extra_instructions = [
            (NavigationSystem.GotoInstructionId, NavigationSystem.goInstruction),
            (ClawProcessor.GrabInstructionTag, ClawProcessor.grabInstruction),
            (ClawProcessor.DropInstructionTag, ClawProcessor.dropInstrution)
        ]
        ScriptProcessor = ScriptSystem.init(extra_instructions, [ClawProcessor.ClawDoneTag])
        self.simulation.add_des_system((ScriptProcessor,),)
        self.simulation.add_des_system((ClawProcessor.process,))
        self.simulation.add_des_system((ObjectManager.process,))

    def add_detection_ability(self):
        """Adds to the system the ability to detect entities using a Camera component."""
        self.simulation.add_des_system((SensorSystem.init(Camera, 0.1),))

    def add_approximation_ability(self):
        """Adds to the system the ability to approach entities detected by the camera."""
        self.simulation.add_des_system((ApproximationProcessor.process,))

    def add_camera(self, drawio_id):
        """Adds a Camera component to entity drawio_id."""
        camera = Camera()
        camera.reply_channel = Store(self.simulation.ENV)
        self.add_component(camera, drawio_id)

    def add_camera_detection_event(self, entity_id, target_id):
        """
        Adds the process_camera_event system to the environment to detect
        colissions between entity_id and target_id.
        """
        camera: Camera = self.get_component(Camera, entity_id)
        target = self.cast_id(target_id)
        self.simulation.ENV.process(process_camera_event(camera, target, self.simulation))

    def add_poi(self, poi_tag: str, poi_value):
        """
        Adds a new POI to the world.

        - poi_tag: the required name for the new POI.
        - poi_value: a tuple with x-axis value and y-axis value. Ex.: (2.5, 3).
        """
        world_map = self.simulation.world.component_for_entity(1, Map)
        world_map.pois[poi_tag] = poi_value

    def add_commands(self, command_list: List, drawio_id: str):
        """
        Adds a list of commands to a robot.

        - Command format: "Go poi_tag", "Grab pickable_name", "Drop pickable_name"
        """
        script = self.get_component(Script, drawio_id)
        if script is None:
            self.add_component(Script(command_list), drawio_id)
        else:
            for command in command_list:
                script.instructions.append(command)

    def add_command(self, command: str, drawio_id: str):
        """
        Adds one command to the robot.

        - Command format: "Go poi_tag", "Grab pickable_name, "Drop pickable_name"
        """
        script = self.get_component(Script, drawio_id)
        if script is None:
            self.add_component(Script([command]), drawio_id)
        else:
            script.instructions.append(command)

    def add_go_command(self, drawio_id: str, poi_tag: str):
        """
        Adds a Go command to a specific poi.

        - drawio_id: property id from an element of the drawio file.
        - poi_tag: the poi_tag property of a POI.
        """
        command = f"Go {poi_tag}"
        self.add_command(command, drawio_id)

    def add_grab_command(self, drawio_id: str, pickable_name: str):
        """
        Adds a Grab pickable command.

        - drawio_id: property id from an element of the drawio file.
        - pickable_name: the name property of the grabbed object, typed 'pickable'.
        """
        command = f"Grab {pickable_name}"
        self.add_command(command, drawio_id)

    def add_drop_command(self, drawio_id: str, pickable_name: str):
        """
        Adds a Drop pickable command.

        - drawio_id: property id from an element of the drawio file.
        - pickable_name: the name property of the grabbed object, typed 'pickable'.
        """
        command = f"Drop {pickable_name}"
        self.add_command(command, drawio_id)

    def add_seer(self, consumer):
        self.simulation.add_des_system(Seer.init([consumer], 0.05, False),)