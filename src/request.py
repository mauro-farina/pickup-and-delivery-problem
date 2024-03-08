from src.graph import Node


class Request:

    def __init__(self, pickup: Node, destination: Node, load: int):
        self.pickup = pickup
        self.destination = destination
        self.load = int(load)
