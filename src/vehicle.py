from src.graph import Node


class Vehicle:

    def __init__(self, index: int, origin_depot: Node, destination_depot: Node,
                 capacity: int, travel_unit_cost: int = 1):
        self.index = index
        self.origin = origin_depot
        self.dest = destination_depot
        self.capacity = capacity
        self.travel_unit_cost = travel_unit_cost

    def __hash__(self):
        return hash((self.index, self.origin, self.dest, self.capacity))

    def __eq__(self, other):
        if other.__class__ is self.__class__:
            return self.index == other.index and self.origin == other.origin and self.dest == other.dest
        else:
            return False

    def __str__(self):
        return f'vehicle {self.index}: origin={self.origin}, destination={self.dest}, capacity={self.capacity}'
