"""
Entrypoint for a simulator.
Defines Simulator class and EVENT.
"""
import json
import simpy
import pathlib
import esper
import logging
import logging.config
import yaml

import typing
import map_parser

from components.Map import Map
from components.Script import Script
from components.Inventory import Inventory

from typehints.dict_types import SystemArgs, Config

fileName = pathlib.Path.cwd().joinpath('loggerConfig.yml')
stream = open(fileName)
loggerConfig = yaml.safe_load(stream)
logging.config.dictConfig(loggerConfig)
logger = logging.getLogger(__name__)

EVENT = typing.NamedTuple('Event', [('type', str), ('payload', typing.NamedTuple)])
"""Format for all DES events added to Stores.

Events are created by systems to exchange messages with other systems.
Arguments:
    type -- identifier of the message.
    payload -- The content of the message. 
    It is defined by the receiving system.
"""

CleanupFunction = typing.Optional[typing.Callable[[], None]]
ConfigFormat = typing.Optional[typing.Union[str, Config]]

SimpyGenerator = typing.Callable[[typing.Generator[simpy.Event, typing.Any, typing.Any]], simpy.Process]
SystemProcessFunction = typing.Callable[[SystemArgs], SimpyGenerator]
DESSystem = typing.Tuple[SystemProcessFunction, typing.Optional[CleanupFunction]]


class Simulator:
    """
    Abstracts the simulator and its controls.

    Simulations are defined via config objects, which is a json-like object.

    Keyword Arguments:
        config: Optional[Union[str, dict]] -- Defines the configuration for the simulator. Either a dict or a path to a json file
        cleanup: Optional[Callable[[], None]] -- Function that can be passed. It's executed after the simulator exits.

    Attributes:
        CONFIG: str -- Either the path to the config file or 'dict object' if the config was passed via dict
        FPS: int -- Speed of step in non-DES systems. Default is 60.
        DEFAULT_LINE_WIDTH: int -- Line width for draw.io models. Might affect size width of objects. Default is 10px.
        DURATION: int -- Optional. Duration of the simulation in seconds.
                         After DURATION seconds the simulation loops exits, but events in the queue are still processed,
                         so final clock on the simulation might be bigger than DURATION seconds.
        EXIT: bool -- Controls if the simulation should exit
    """
    def __init__(self, config: ConfigFormat = None, cleanup: CleanupFunction = lambda: print("Simulator Exited")):
        """
        Load simulation parameters from config.
        Create simulation objects from map, populating them with components.
        """
        self.CONFIG = 'simulation.json' if config is None else config
        if type(self.CONFIG) == str:
            with open(self.CONFIG) as fd:
                config = json.load(fd)
        else:
            self.CONFIG = 'dict object'

        self.FPS = config.get('FPS', 60)
        self.DEFAULT_LINE_WIDTH = config.get('DLW', 10)
        self.DURATION = config.get('duration', -1)

        file = pathlib.Path(config.get('context', '.')) / config.get('map', 'map.drawio')
        simulation = map_parser.build_simulation_from_map(file)
        self.world: esper.World = simulation['world']
        # self.window = simulation['window']
        # self.batch = simulation['batch']
        _, self.window_dimensions, _ = simulation['window_props']
        self.draw2ent = simulation['draw_map']
        self.objects = simulation['objects']
        # Global inventory
        self.interactive = self.world.component_for_entity(1, Inventory).objects

        logger.info(f"==> Simulation objects")
        for oid, objId in self.objects:
            entity = self.draw2ent.get(objId)
            logger.info(f"OBJ #{oid} (draw {objId}). Type {entity[1]['type']}")
            if self.world.has_component(oid, Map):
                ent_map = self.world.component_for_entity(oid, Map)
                logger.info("\tAvailable paths:")
                for idx, key in enumerate(ent_map.paths.keys()):
                    logger.info(f"\t{idx}. {key}")
            if self.world.has_component(oid, Script):
                script = self.world.component_for_entity(oid, Script)
                logger.info(script)

        self.EXIT: bool = False
        self.ENV = simpy.Environment()
        self.EXIT_EVENT = self.ENV.event()
        self.KWARGS: SystemArgs = {
            "ENV": self.ENV,
            "WORLD": self.world,
            "_KILLSWITCH": self.ENV.event() if self.DURATION > 0 else None,
            "EVENT_STORE": simpy.FilterStore(self.ENV),
            # Pyglet specific things (for the re-create entity)
            # "BATCH": self.batch,
            # "WINDOW_OPTIONS": (self.window_dimensions, self.DEFAULT_LINE_WIDTH),
        }
        self.cleanups: typing.List[CleanupFunction] = [cleanup]

    def add_DES_system(self, system: DESSystem):
        """
        Adds a Discrete system to the simulation environment.
        Discrete systems use simpy's DES mechanisms to wait events that trigger their processing,
        rather than being executed at every simulation step.
        DES systems can also inform a cleanup function. It will be executed when the simulator exits.
        """
        self.ENV.process(system[0](kwargs=self.KWARGS))
        if len(system) == 2:
            self.cleanups.append(system[1])

    def add_system(self, system):
        """
        Adds an esper system to the simulation environment.
        These events inherit from esper.Processor and are executed at every simulation step.
        An argument kwargs: SystemArgs will be passed to these processors.
        """
        self.world.add_processor(system)

    def simulation_loop(self):
        """
        The simulation loop
        """
        # Other processors
        while not self.EXIT:
            # pyglet.clock.tick()
            self.world.process(self.KWARGS)
            # logger.debug(f'[ENV TIME {self.ENV.now}] Processed world.')
            # For many windows
            # for w in pyglet.app.windows:
            #     w.switch_to()
            #     w.dispatch_events()
            #     w.dispatch_event('on_draw')
            #     w.flip()
            # # ticks on the clock
            if self.KWARGS["_KILLSWITCH"] is not None:
                switch = yield self.KWARGS["_KILLSWITCH"] | self.ENV.timeout(1.0 / self.FPS, False)
                if self.KWARGS["_KILLSWITCH"] in switch:
                    break
            else:
                yield self.ENV.timeout(1.0 / self.FPS, False)
        return 0

    def run(self):
        """
        Runs the simulation.
        Simulation continues for DURATION seconds, if DURATION is specified.
        Simulation exits on EXIT_EVENT or if _KILLSWITCH event is triggered.
        After simulation loop terminates, ALL cleanup functions are executed,
        the last being the user defined one, if present.
        """
        if self.DURATION > 0:
            self.ENV.process(self.simulation_loop())
            self.ENV.run(until=self.DURATION)
        else:
            self.ENV.process(self.simulation_loop())
            self.ENV.run(until=self.EXIT_EVENT)
        while self.cleanups:
            next_function = self.cleanups.pop()
            next_function()
