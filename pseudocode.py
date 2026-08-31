import sys
print('why are you running the pseudocode')
sys.exit()

from enum import Enum
from typing import List


class Direction(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

class Node:
    def get_neighbour(self, direction: Direction) -> 'Node':
        ...


def look_around() -> List[Direction]:
    ...

def move_to(neighbour: Node):
    ... # For now

def move_back():
    ... # For now


def dfs(node: Node):
    visited = set()
    visited.add(node)
    
    open_directions = look_around()

    for d in open_directions:
        neighbour = node.get_neighbour(d)

        if neighbour in visited:
            continue

        visited.add(neighbour)

        move_to(neighbour)

        dfs(neighbour)

        move_back()


if __name__ == "__main__":
    dfs(Node())