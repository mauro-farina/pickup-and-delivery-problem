from src.network.graph import Node
from dataclasses import dataclass


@dataclass(frozen=True)
class Request:
    index: int
    pickup: Node
    destination: Node
    load: int
