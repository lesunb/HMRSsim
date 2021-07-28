import typing
import esper
import simpy
import enum

from simulator.typehints.build_types import WindowOptions


class SystemArgs(typing.TypedDict):
    """Type of Keyword Arguments passed to systems in the process method."""
    ENV: simpy.Environment
    WORLD: esper.World
    _KILL_SWITCH: typing.Union[simpy.Event, None]
    EVENT_STORE: simpy.FilterStore
    WINDOW_OPTIONS: WindowOptions


class EntityDefinition(typing.TypedDict):
    entId: str
    components: typing.Dict[str, list]
    isObject: bool
    isInteractive: bool
    name: typing.Optional[str]
    type: typing.Optional[str]

class LogLevel(enum.Enum):
    DEBUG = 10
    INFO  = 20
    SEER  = 25
    WARN  = 30
    ERROR = 40

    def __lt__(self, other):
        if isinstance(other, LogLevel):
            return self.value < other.value
        elif isinstance(other, int):
            return self.value < other
        else:
            raise TypeError(f"'<' not supported between instances of 'LogLevel' and {type(other)}")

    def __ge__(self, other):
        if isinstance(other, LogLevel):
            return self.value >= other.value
        elif isinstance(other, int):
            return self.value >= other
        else:
            raise TypeError(f"'>=' not supported between instances of 'LogLevel' and {type(other)}")

class SimulatorOptions(typing.TypedDict):
    loggerConfig: typing.Optional[str]

class Config(typing.TypedDict):
    """Options for the Simulation config

        Arguments:
            context: str -- Change the base directory for simulation assets. Default is .
            map: str -- Name of simulation map file. Must be under assets folder. Default is 'map.drawio'
    """
    context: str
    map: typing.Optional[str]
    FPS: typing.Optional[int]
    DLW: typing.Optional[int]
    duration: typing.Optional[int]
    verbose: typing.Optional[typing.Union[LogLevel, int]]
    simulationComponents: typing.Optional[typing.Dict[str, list]]
    extraEntities: typing.Optional[typing.List[EntityDefinition]]
    simulatorConfigOptions: typing.Optional[SimulatorOptions]
