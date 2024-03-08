from enum import Enum


class NodeType(Enum):
    ORIGIN_DEPOT = 1
    DESTINATION_DEPOT = 2
    PICKUP = 3
    DELIVERY = 4
    TRANSFER_STATION = 5


class Node:

    def __init__(self, name: str, node_type: NodeType):
        self.name = name
        self.type = node_type


class Arc:

    def __init__(self, source: Node, destination: Node, cost: float):
        self.source = source
        self.destination = destination
        self.cost = cost


class Graph:

    def __init__(self, nodes: set[Node], arcs: set[Arc]):
        self.nodes = nodes
        self.arcs = arcs
