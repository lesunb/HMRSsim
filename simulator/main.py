"""
Entrypoint for a simulator.
Defines Simulator class.
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

from components.Inventory import Inventory
from utils.create_components import initialize_components
from typehints.dict_types import SystemArgs, Config, EntityDefinition

fileName = pathlib.Path.cwd().joinpath('loggerConfig.yml')
stream = open(fileName)
loggerConfig = yaml.safe_load(stream)
logging.config.dictConfig(loggerConfig)
logger = logging.getLogger(__name__)


"""Format for all DES events added to Stores.

Events are created by systems to exchange messages with other systems.
Arguments:
    type -- identifier of the message.
    payload -- The content of the message. 
    It is defined by the receiving system.
"""

CleanupFunction = typing.Optional[typing.Callable[[], None]]
ConfigFormat = typing.Optional[typing.Union[str, Config]]

SimpyEvent = typing.Generator[simpy.Event, typing.Any, typing.Any]
SystemProcessFunction = typing.Callable[[SystemArgs], SimpyEvent]
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
        logger.info('========== SIMULATION LOADING ==========')
        self.CONFIG = 'simulation.json' if config is None else config
        if type(self.CONFIG) == str:
            with open(self.CONFIG) as fd:
                config = json.load(fd)
        else:
            self.CONFIG = 'dict object'
        logger.info(f'Loading simulation from {self.CONFIG}')

        self.FPS = config.get('FPS', 60)
        self.DEFAULT_LINE_WIDTH = config.get('DLW', 10)
        self.DURATION = config.get('duration', -1)

        if 'map' in config:
            file = pathlib.Path(config.get('context', '.')) / config.get('map')
            logger.info(f'Using simulation map {file}')
            simulation = map_parser.build_simulation_from_map(file)
        else:
            logger.info('No map found in the configuration. Creating empty simulation.')
            simulation = map_parser.build_simulation_from_map("", True)
        self.world: esper.World = simulation['world']
        # self.window = simulation['window']
        # self.batch = simulation['batch']
        self.simulation_name, self.window_dimensions, _ = simulation['window_props']
        self.draw2ent = simulation['draw_map']
        self.objects = simulation['objects']
        # Global inventory
        self.interactive = self.world.component_for_entity(1, Inventory).objects

        extra_entities = config.get('extraEntities', None)
        if extra_entities is not None:
            logger.info(f'Loading extra entities from config')
            extras = 0
            for entity_definition in extra_entities:
                ent_id = entity_definition.get('entId', f'extraEntity_{extras}')
                extras += 1
                self.add_entity(entity_definition, ent_id)

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
            "WINDOW_OPTIONS": (self.window_dimensions, self.DEFAULT_LINE_WIDTH),
        }
        self.cleanups: typing.List[CleanupFunction] = [cleanup]
        self.generate_simulation_build_report()

    def generate_simulation_build_report(self):
        logger.info('===== SIMULATION LOADING COMPLETE =====')
        logger.info(f'Simulation {self.simulation_name}')
        logger.info(f'===> Simulation components')
        for c in self.world.components_for_entity(1):
            logger.info(c)
        logger.info(f'{len(self.draw2ent)} entities created.')
        logger.info(f'{len(self.objects)} typed objects transformed into entities')
        logger.info(f'===> TYPED OBJECTS')
        for k, v in self.draw2ent.items():
            if v[1].get('type', None) is None:
                continue
            logger.info(f'• {k} --> esper entity {v[0]}. (type {v[1].get("type", "")})')
            ent = v[0]
            components = self.world.components_for_entity(ent)
            logger.info(f'Entity has {len(components)} components.')
            for c in components:
                logger.info(f'\t- {c}')
        logger.info(f'===> Interactive objects')
        logger.info(self.interactive)

    def add_des_system(self, system: DESSystem):
        """
        Adds a Discrete system to the simulation environment.
        Discrete systems use simpy's DES mechanisms to wait events that trigger their processing,
        rather than being executed at every simulation step.
        DES systems can also inform a cleanup function. It will be executed when the simulator exits.
        """
        process_function: SystemProcessFunction = system[0]
        self.ENV.process(process_function(self.KWARGS))
        if len(system) == 2:
            self.cleanups.append(system[1])

    def add_system(self, system):
        """
        Adds an esper system to the simulation environment.
        These events inherit from esper.Processor and are executed at every simulation step.
        An argument kwargs: SystemArgs will be passed to these processors.
        """
        self.world.add_processor(system)

    def add_entity(self, entity_definition: EntityDefinition, ent_id: str):
        """Add an entity from json to world."""
        initialized_components = initialize_components(entity_definition.get('components', {}))
        ent = self.world.create_entity(*initialized_components)
        self.draw2ent[ent_id] = [ent, {}]
        if entity_definition.get('isInteractive', False):
            self.interactive[entity_definition.get('name', ent_id)] = ent
        if entity_definition.get('isObject', False):
            self.objects.append((ent, ent_id))

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
        logger.info('============ SIMULATION EXECUTION ============')
        if self.DURATION > 0:
            self.ENV.process(self.simulation_loop())
            self.ENV.run(until=self.DURATION)
        else:
            self.ENV.process(self.simulation_loop())
            self.ENV.run(until=self.EXIT_EVENT)
        logger.info('============ SIMULATION EXECUTION FINISHED ============')
        logger.info('============ CLEAN UP STAGE ============')
        logger.info(f'{len(self.cleanups)} Clean up functions to execute')
        while self.cleanups:
            next_function = self.cleanups.pop()
            logger.info(f'Executing clean-up function {next_function}')
            next_function()
