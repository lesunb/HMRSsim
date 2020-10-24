from enum import Enum

from typing import List


class States(Enum):
    READY = 'ready'
    BLOQUED = 'bloqued'
    DONE = 'done'

class Script:

    def __init__(self, instructions: List[str], delay: int):
        self.curr_instruction = 0
        self.instructions = instructions
        self.state = States.READY
        self.delay = delay
        self.logs = []
        self.expecting = []

    def __str__(self):
        sep = '\n'
        return f'Script[{self.curr_instruction}/{len(self.instructions)} - {self.state}]:\n{sep.join(self.instructions)}'
