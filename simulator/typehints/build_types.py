import typing

WindowOptions = typing.Tuple[typing.Tuple[int, int], int]


class DependencyNotFound(Exception):
    pass
