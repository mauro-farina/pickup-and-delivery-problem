from enum import StrEnum


class NodeType(StrEnum):
    ORIGIN_DEPOT = 'o'
    DESTINATION_DEPOT = 'e'
    PICKUP = 'p'
    DELIVERY = 'd'
    TRANSFER_STATION = 't'


class Node:

    def __init__(self, index: int, node_type: NodeType, coordinates: tuple[int, int],
                 earliest_time: int, latest_time: int):
        self.index = index
        self.type = node_type
        self.coordinates = coordinates
        self.earliest_time = earliest_time
        self.latest_time = latest_time

    def __eq__(self, other):
        if other.__class__ is self.__class__:
            return (self.index == other.index
                    and self.type == other.type
                    and self.coordinates == other.coordinates)
        else:
            return False

    def __hash__(self):
        return hash(self.index)

    def __str__(self):
        return f'{self.type}{self.index}@{self.coordinates};E={self.earliest_time};L={self.latest_time}'


class Arc:

    def __init__(self, source: Node, destination: Node, cost: float):
        self.src = source
        self.dst = destination
        self.cost = cost


class Graph:

    def __init__(self, nodes: set[Node], arcs: set[Arc]):
        self.nodes = nodes
        self.arcs = arcs

    def get_arc(self, source_index: int, destination_index: int) -> None | Arc:
        for arc in self.arcs:
            if (arc.src.index == source_index
                    and arc.dst.index == destination_index):
                return arc
        return None
