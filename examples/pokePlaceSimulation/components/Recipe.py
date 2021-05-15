from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class Recipe:
    name: str
    prep_time: float
    ingredients: List[Tuple[str, float]]
    usesStove: bool

