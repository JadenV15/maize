#!/usr/bin/env python3

# FOLLOW THE PSEUDOCODE, OR ELSE!

from typing import List, Optional

from constants import *
from utils import *

from robot import Robot
from map import Tile, Edge, Map


# PSEUDO HERE:


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
        if neighbour not in visited:
            visited.add(neighbour)
            move_to(neighbour)
            dfs(neighbour)
            move_back()

if __name__ == "__main__":
    dfs(Node()) 

