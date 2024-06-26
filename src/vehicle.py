from dataclasses import dataclass

from graph import Node


@dataclass(frozen=True)
class Vehicle:
    index: int
    origin: Node
    dest: Node
    capacity: int
    travel_unit_cost: int = 1
