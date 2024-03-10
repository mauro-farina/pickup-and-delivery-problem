from src.graph import Node


class Request:

    def __init__(self, index: int, pickup: Node, destination: Node, load: int):
        self.index = index
        self.pickup = pickup
        self.destination = destination
        self.load = int(load)
