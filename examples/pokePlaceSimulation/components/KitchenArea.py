from dataclasses import dataclass, field
from typing import Tuple
from simpy import Resource


@dataclass
class KitchenArea:
    name: str
    max_resources: int
    position: Tuple[float, float]
    resource: Resource = None

    def __str__(self):
        return f'KitchenArea<{self.name}>[max_resources={self.max_resources}]'
