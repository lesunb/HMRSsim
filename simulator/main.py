"""
Entrypoint for a simulator.
Defines Simulator class and EVENT.
"""
import json
import simpy
import pathlib
import pyglet
import esper
import logging
import logging.config
import yaml

from typing import NamedTuple

import map_parser

from components.Map import Map
from components.Script import Script
from components.Inventory import Inventory

fileName = pathlib.Path.cwd().joinpath('loggerConfig.yml')
stream = open(fileName)
loggerConfig = yaml.safe_load(stream)
logging.config.dictConfig(loggerConfig)
logger = logging.getLogger(__name__)

EVENT = NamedTuple('Event', type=str, payload=object)


class Simulator:
    """
    class Simulator abstracts the simulator and its controls.
    Simulations are defined via config objects, which is a json file.
    """
    def __init__(self, config=None, cleanup=lambda: print("Simulator Exited")):
        """
        Loads simulation parameters from config.
        Creates simulation objects from map, populating them with components.
        """
        self.CONFIG = 'simulation.json' if config is None else config
        with open(self.CONFIG) as fd:
            config = json.load(fd)
            self.FPS = config.get('FPS', 60)
            self.DEFAULT_LINE_WIDTH = config.get('DLW', 10)
            file = pathlib.Path(config.get('context', '.')) / config.get('map', 'map.drawio')
            self.duration = config.get('duration', -1)

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

        self.EXIT = False
        self.ENV = simpy.Environment()
        self.EXIT_EVENT = self.ENV.event()
        self.KWARGS = {
            "ENV": self.ENV,
            "WORLD": self.world,
            "_KILLSWITCH": self.ENV.event() if self.duration > 0 else None,
            "EVENT_STORE": simpy.FilterStore(self.ENV),
            # Pyglet specific things (for the re-create entity)
            # "BATCH": self.batch,
            "WINDOW_OPTIONS": (self.window_dimensions, self.DEFAULT_LINE_WIDTH),
        }
        self.cleanups = [cleanup]

    def add_DES_system(self, system):
        """
        Adds a Discrete system to the simulation environment.
        Discrete systems use simpy's DES mechanisms to wait events that trigger their processing,
        rather than being executed at every simulation step.
        Such systems must have a generator function with the following signature:
            def process(kwargs: dict) -> None:
        """
        self.ENV.process(system[0](kwargs=self.KWARGS))
        if len(system) == 2:
            self.cleanups.append(system[1])

    def add_system(self, system):
        """
        Adds an esper system to the simulation environment.
        These events inherit from esper.Processor and are executed at every simulation step.
        An argument kwargs will be passed to these processors
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
        Starts the simulation
        """
        if self.duration > 0:
            self.ENV.process(self.simulation_loop())
            self.ENV.run(until=self.duration)
        else:
            self.ENV.process(self.simulation_loop())
            self.ENV.run(until=self.EXIT_EVENT)
        while self.cleanups:
            next_function = self.cleanups.pop()
            next_function()
