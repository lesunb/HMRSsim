from dataclasses import dataclass, field
from typing import Dict
from examples.pokePlaceSimulation.components.Recipe import Recipe


@dataclass
class Menu:
    items: Dict[str, Recipe] = field(default_factory=dict)

    def __str__(self):
        return f'Menu[{list(self.items.keys())}]'
