#!/usr/bin/env python3

from constants import *
from utils import *

from robot import Robot
from map import Tile, Edge, Map

__all__ = ['dfs']


def move(current: Tile, target: Tile, relative_direction: Direction):
    ...


def dfs(robot: Robot, map: Map, tile: Tile):
    """Entry point for dfs.
    For the first call, the robot is expected to be in the tile north of the start tile,
    origin at the tile origin,
    facing global north.
    This func is called recursively.
    NOTE: <tile> always refers to the tile the robot is ON, NOT the one it's GOING TO.
    """
    # mark this tile as visited (we are ON it)
    # technically, all tiles in map are visited. This is just a bit more explicit, i guess
    assert not tile.visited
    tile.visited = True

    # since the robot is at the centre of  `tile`,
    # take a reading of the tile type
    # and assign it to the tile
    tile_type = robot.tile_type
    assert tile_type not in (TileType.START, TileType.NOGO)
    tile.tile_type = tile_type

    # scan open directions
    open_directions = robot.lookaround()

    for rel_direction in open_directions:
        # NOTE: this direction is RELATIVE. we need to convert it to a GLOBAL on.
        global_direction = Direction(rotate(rel_direction, robot.heading))

        # get neighbour in that direction
        neighbour = map.get_tile_by_direction(tile, global_direction)

        # check if already visited
        if neighbour is not None and neighbour.visited:
            continue # next

        neighbour = Tile(tile_point=shift(
            tile.tile_point, global_direction
        ))

        # add neighbour
        map.add_tile(neighbour)
        map.mark_open(tile, neighbour)

        move(tile, neighbour, rel_direction)

        dfs(robot, map, neighbour)

        move(neighbour, tile, rel_direction.reverse())

    