from src.graph import Node


class Vehicle:

    def __init__(self, index: int, origin_depot: Node, destination_depot: Node,
                 capacity: int, travel_unit_cost: int = 1):
        self.index = index
        self.origin = origin_depot
        self.dest = destination_depot
        self.capacity = capacity
        self.travel_unit_cost = travel_unit_cost
