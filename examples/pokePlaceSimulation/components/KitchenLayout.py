from dataclasses import dataclass, field
from typing import Dict, List, NamedTuple
from components.KitchenArea import KitchenArea
from simpy import FilterStore

OpenOrder = NamedTuple('OpenOrder', [('orderId', int), ('createdAt', float), ('stove', bool), ('items', List[str])])


@dataclass
class KitchenLayout:
    personnel: int
    areas: Dict[str, KitchenArea] = field(default_factory=dict)
    next_order: int = 0
    avg_time_per_order: int = 0
    orders_processed: int = 0

    def __str__(self):
        return f'KitchenLayout[personnel={self.personnel}; areas={list(self.areas.keys())}]'
