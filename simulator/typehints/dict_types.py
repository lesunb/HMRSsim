import typing
import esper
import simpy

from typehints.build_types import WindowOptions


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
    simulationComponents: typing.Optional[typing.Dict[str, list]]
    extraEntities: typing.Optional[typing.List[EntityDefinition]]
