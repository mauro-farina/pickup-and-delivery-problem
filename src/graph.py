from enum import StrEnum


class NodeType(StrEnum):
    ORIGIN_DEPOT = 'o'
    DESTINATION_DEPOT = 'e'
    PICKUP = 'p'
    DELIVERY = 'd'
    TRANSFER_STATION = 't'


class Node:

    def __init__(self, index: int, node_type: NodeType):
        self.index = index
        self.type = node_type

    def __eq__(self, other):
        if other.__class__ is self.__class__:
            return self.index == other.index and self.type == other.type
        else:
            return False

    def __hash__(self):
        return hash(self.index)


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
