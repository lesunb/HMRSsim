import typing

WindowOptions = typing.Tuple[typing.Tuple[int, int], int]

class DependencyNotFound(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)

class SimulationParseError(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)

class ConfigParseError(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)