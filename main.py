#!/usr/bin/env python3

# FOLLOW THE PSEUDOCODE, OR ELSE!

from typing import List, Optional

from constants import *
from utils import *

from robot import Robot
from map import Tile, Edge, Map
from move import dfs


def main():
    # create the Robot
    robot = Robot()
    robot.init()

    try:

        # current position:
        '''
        '@' represents the robot origin
        '%' represents the tile origin
        '!' represents the tile south midpoint
        +----(0, 0)-----+  ← tile point written there
        |               |
        |    +--@--+    |
        |    |  %  |    |
        |    |     |    |
        |    |     |    |
        +----+--!--+----+
        '''

        map = Map() # hmm, shadowing

        # add current tile
        start_tile = Tile(
            tile_point=Point(0, 0), # define the global origin to be at the start tile origin
            tile_type=TileType.START, # this is a Start tile
            visited=True # we have 'visited' the start tile
        )
        map.add_tile(start_tile)

        # calibrate the robot origin to the actual origin
        # the below var represents '@' in the diagram. We find it by shifting up from '!' by <robot height>
        origin = shift(
            shift(start_tile.origin, Direction.SOUTH, TILE_HALF_WIDTH), # '!' in the above diagram
            Direction.NORTH,
            ROBOT_HEIGHT
        )
        robot.origin = origin

        # assert the current tile is a start tile
        assert robot.tile_type == TileType.START


        # move the robot to the next tile, and centre its origin
        # there is no need to worry about the next tile being black
        # aim:
        '''
        '@' represents both the robot origin and the tile origin
        '&' represents the old robot origin, from the above diagram
        +----(0, 1)-----+
        |               |
        |               |
        |    +--@--+    |  next_tile
        |    |     |    |
        |    |     |    |  ↑
        +----|     |----+  ↑
        |    +-----+    |  ↑ forward
        |    +--&--+    |  ↑ movement
        |    |     |    |  ↑
        |    |     |    |  ↑
        |    |     |    |
        +----+--!--+----+  start_tile
        '''

        # define the tile north of the start tile
        # which is (0, 1)
        next_tile = Tile(tile_point=Point(0, 1))
        map.add_tile(next_tile)
        map.mark_open(start_tile, next_tile)

        # move
        delta = origin.distance(next_tile.origin) # distance to move forward by
        print(delta)
        robot.drive(delta)
        #next_tile.visited = True
        # ^^^ don't do this, as dfs() will mark it as visited

        # add tile type of next_tile
        tile_type = robot.tile_type
        assert tile_type not in (TileType.START, TileType.NOGO)
        next_tile.tile_type = tile_type

        # at this point we are set up
        wait()

        # Exit, no dfs - I just want to test above code for now
        raise SystemExit

        # start exploring from tile (0,1)
        dfs(robot, map, next_tile)

        wait()

        # finally, move back to the start tile and align against back wall
        robot.drive(delta, forward=False)

        #TODO


    finally:
        robot.shutdown()


if __name__ == '__main__':
    input('Press enter to start...') #TODO :remove

    main()
