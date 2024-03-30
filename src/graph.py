from enum import StrEnum
from dataclasses import dataclass


class NodeType(StrEnum):
    ORIGIN_DEPOT = 'o'
    DESTINATION_DEPOT = 'e'
    PICKUP = 'p'
    DELIVERY = 'd'
    TRANSFER_STATION = 't'


@dataclass(frozen=True)
class Node:
    index: int
    type: NodeType
    coordinates: tuple[int, int]
    earliest_time: int
    latest_time: int


@dataclass(frozen=True)
class Arc:
    src: Node
    dst: Node
    cost: float


@dataclass(frozen=True)
class Graph:
    nodes: set[Node]
    arcs: set[Arc]
