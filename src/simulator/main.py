"""
Entrypoint for a simulator.
Defines Simulator class.
"""
import json
import yaml
import simpy
import pathlib
import esper
from logging import getLogger
import logging.config
from simulator.logger_config import logger_config

import typing
from simulator import map_parser

from datetime import datetime, timedelta
from pathlib import Path

from simulator.components.Inventory import Inventory
from simulator.typehints.build_types import SimulationParseError
from simulator.utils.create_components import initialize_components, import_external_component
from simulator.typehints.dict_types import LogLevel, SystemArgs, Config, EntityDefinition
from simulator.utils.validators import validate_config

logging.config.dictConfig(logger_config)
logger = getLogger(__name__)
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
        FPS: int -- Speed of step in non-DES systems. If missing, no non-DES system is executed
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
        self.build_report = []
        self.build_report.append('========== SIMULATION LOADING ==========')
        self.CONFIG = 'simulation.json' if config is None else config
        if type(self.CONFIG) == str:
            with open(self.CONFIG) as fd:
                config = json.load(fd)
        else:
            self.CONFIG = 'dict object'
        self.build_report.append(f'Validating config...')
        config_errors = validate_config(config)
        if len(config_errors) > 0:
            self.build_report += config_errors
            logger.error(f'Failed to parse config from {self.CONFIG}')
            logger.error(f'{len(config_errors)} errors found in config:')
            logger.error('\n- '.join(config_errors))
            logger.error('Simulation execution aborted')
            raise SimulationParseError(f'Config from {self.CONFIG} could not be parsed.')
        else:
            self.build_report.append('Config OK ✔')
        self.build_report.append(f'Loading simulation from {self.CONFIG}')
        # Parse level of verbosity.
        # Can be an int or a LogLevel
        self.verbose: LogLevel = config.get('verbose', LogLevel.ERROR)
        if isinstance(self.verbose, LogLevel):
            logger.root.setLevel(self.verbose.value)
            logger.setLevel(self.verbose.value)
        elif isinstance(self.verbose, int):
            logger.root.setLevel(self.verbose)
            logger.setLevel(self.verbose)

        self.FPS = config.get('FPS', 0) if config is not None else 0
        if self.FPS < 0:
            logger.warning(f'WARNING: FPS value should not be negative')
            self.build_report.append(f'WARNING: FPS value should not be negative')
            self.FPS = 0
        self.DEFAULT_LINE_WIDTH = config.get('DLW', 10) if config is not None else 10
        self.DURATION = config.get('duration', -1) if config is not None else -1
        simulation_components = config.get('simulationComponents', None) if config is not None else None

        context = config.get('context', '.') if config is not None else '.'
        self.build_report.append(f'Context is {context} ({Path(context).absolute()})')
        # Check for extra config options
        self.simulator_extra_config = config.get('simulatorConfigOptions', {})
        if ('loggerConfig' in self.simulator_extra_config):
            logger_config_file = Path(context) / self.simulator_extra_config['loggerConfig']
            if logger_config_file.exists():
                logger.info(f'Loading logger config file {logger_config_file.absolute()}')
                self.build_report.append(f'Loading logger config file {logger_config_file.absolute()}')
                with open(logger_config_file) as fd:
                    logger_config = yaml.safe_load(fd)
                    logging.config.dictConfig(logger_config)
            else:
                logger.error(f'Logger config file {logger_config_file.absolute()} not found')
                self.build_report.append(f'ERROR: Logger config file {logger_config_file.absolute()} not found')

        import_external_component(context)
        if 'map' in config:
            file = pathlib.Path(context) / config.get('map')
            self.build_report.append(f'Using simulation map {file}')
            simulation = map_parser.build_simulation_from_map(file, simulation_components)
        else:
            self.build_report.append('No map found in the configuration. Creating empty simulation.')
            simulation = map_parser.build_simulation_from_map(context, simulation_components, True)
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
            self.build_report.append(f'Loading extra entities from config')
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
            "OBJECTS": self.objects,
            "DRAW2ENT": self.draw2ent,
            "INTERACTIVE": self.interactive,
            "_KILL_SWITCH": self.EXIT_EVENT,
            "EVENT_STORE": simpy.FilterStore(self.ENV),
            "WINDOW_OPTIONS": (self.window_dimensions, self.DEFAULT_LINE_WIDTH),
        }
        self.cleanups: typing.List[CleanupFunction] = [cleanup]
        self.build_report.append('========== SIMULATION LOADING COMPLETE ==========')
        self.generate_simulation_build_report()
        if LogLevel.WARN >= self.verbose:
            print('\n'.join(map(str.strip, self.build_report)))

    def generate_simulation_build_report(self):
        self.build_report.append(f'Simulation {self.simulation_name}\n')
        self.build_report.append(f'===> Simulation components\n')
        for c in self.world.components_for_entity(1):
            self.build_report.append('- ' + str(c) + '\n')
        self.build_report.append(f'{len(self.draw2ent)} entities created.\n')
        self.build_report.append(f'{len(self.objects)} typed objects transformed into entities\n')
        self.build_report.append(f'===> TYPED OBJECTS\n')
        for k, v in self.draw2ent.items():
            if v[1].get('type', None) is None:
                continue
            self.build_report.append(f'• {k} --> esper entity {v[0]}. (type {v[1].get("type", "")})\n')
            ent = v[0]
            components = self.world.components_for_entity(ent)
            self.build_report.append(f'Entity has {len(components)} components.\n')
            for c in components:
                self.build_report.append(f'\t- {c}\n')
        if len(self.interactive):
            self.build_report.append(f'===> Interactive objects\n')
            self.build_report.append(str(self.interactive) + '\n')

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
        if self.FPS == 0:
            logger.warning(f'Adding non-DES system {system}, but FPS was not given. System is useless.')
            self.build_report.append(f'WARNING: Useless non-DES system {system} because no FPS was provided.')
        self.world.add_processor(system)

    def add_entity(self, entity_definition: EntityDefinition, ent_id: str):
        """Add an entity from json to world."""
        initialized_components = initialize_components(entity_definition.get('components', {}))
        ent = self.world.create_entity(*initialized_components)
        self.draw2ent[ent_id] = [ent, {'type': entity_definition['type']}]
        if entity_definition.get('isInteractive', False):
            self.interactive[entity_definition.get('name', ent_id)] = ent
        if entity_definition.get('isObject', False):
            self.objects.append((ent, ent_id))

    def simulation_loop(self):
        """
        The simulation loop
        """
        # Local ref most used vars
        process_esper_systems = self.world.process
        kill_switch = self.KWARGS['_KILL_SWITCH']
        sleep = self.ENV.timeout
        sleep_interval = 1.0 / self.FPS if self.FPS > 0 else None
        # Collect info
        # Other processors
        while not self.EXIT:
            start = datetime.now()
            if sleep_interval:
                process_esper_systems(self.KWARGS)
            # # ticks on the clock
            if sleep_interval:
                switch = yield kill_switch | sleep(sleep_interval, False)
            else:
                switch = yield kill_switch
            if kill_switch in switch:
                break
        logger.debug(f'simulation loop exited')

    def gracious_exit(self):
        logger.info(f'Exiting gracefully')
        while self.cleanups:
            next_function = self.cleanups.pop()
            logger.info(f'Executing clean-up function {next_function}')
            next_function()

    def run(self):
        """
        Runs the simulation.
        Simulation continues for DURATION seconds, if DURATION is specified.
        Simulation exits on EXIT_EVENT (or if _KILL_SWITCH, they're the same) event is triggered.
        After simulation loop terminates, ALL cleanup functions are executed,
        the last being the user defined one, if present.
        """
        try:
            logger.info('============ SIMULATION EXECUTION ============')
            if self.DURATION > 0:
                self.ENV.process(self.simulation_loop())
                self.ENV.run(until=self.DURATION)
            else:
                self.ENV.process(self.simulation_loop())
                self.ENV.run(until=self.EXIT_EVENT)
            logger.info('============ SIMULATION EXECUTION FINISHED ============')
            logger.info(f'{len(self.cleanups)} Clean up functions to execute')
        except RuntimeError as err:
            logger.error(f'Simulation aborted with critical error.')
            logger.error(err)
        finally:
            logger.info('============ CLEAN UP STAGE ============')
            self.gracious_exit()
