from enum import Enum
from typehints.component_types import Component
from typehints.dict_types import SystemArgs
from typing import List, Dict, Callable, NamedTuple

ErrorHandler = Callable[[NamedTuple, SystemArgs], None]


class States(Enum):
    READY = 'ready'
    BLOCKED = 'blocked'
    DONE = 'done'


class Script(Component):

    def __init__(
            self,
            instructions: List[str],
            error_handlers: Dict[str, ErrorHandler] = dict(),
            default_error_tag: str = '__default',
            delay: int = 0
    ):
        self.curr_instruction = 0
        self.instructions = instructions
        self.state = States.READY
        self.delay = delay
        self.logs = []
        self.expecting = []
        self.error_handlers = error_handlers
        self.default_error_tag = default_error_tag

    def __str__(self):
        r = f'Script[{self.curr_instruction}/{len(self.instructions)} - {self.state}]:\n\t'
        r += '\n\t'.join(self.instructions)
        r += '\n\t' + ("-" * 10)
        return r
