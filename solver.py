#!/usr/bin/env python3

from constants import *
from utils import *

from robot import Robot
from map import Tile, Edge, Map

__all__ = ['Solver']


class BlackTileError(Exception):
    """The robot has detected a black tile while executing a MovementType.
    The robot has moved back to the last position.
    """
    pass


class Solver:
    """mAZE solver"""
    
    def __init__(self, robot: Robot, map: Map):
        self._robot = robot
        self._map = map

        # added in solve():
        # self._current_tile: Tile


    def solve(self, test_initial: bool = False, test_without_calibration: bool = False):
        """Main entry point"""
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
        self._robot.origin = origin

        # assert the current tile is a start tile
        assert self._robot.tile_type == TileType.START


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
        self._robot.drive(delta)
        #next_tile.visited = True
        # ^^^ don't do this, as dfs() will mark it as visited

        # add tile type of next_tile
        tile_type = self._robot.tile_type
        assert tile_type not in (TileType.START, TileType.NOGO)
        next_tile.tile_type = tile_type

        # at this point we are set up
        wait()

        if test_initial:
            # Exit, no dfs - I just want to test above code for now
            raise SystemExit

        # assign current Tile
        self._current_tile = next_tile # type: Tile

        # for testing
        self._calibrate = not test_without_calibration

        # start exploring from tile (0,1)
        self.dfs(next_tile)

        wait()

        # finally, move back to the start tile and align against back wall
        self._robot.drive(delta, forward=False)

        #TODO


    # dfs


    def dfs(self, tile: Tile):
        """Entry point for dfs.
        For the first call, the robot is expected to be in the tile north of the start tile,
        origin at the tile origin,
        facing global north.
        This func is called recursively.
        NOTE: <tile> always refers to the tile the robot is ON, NOT the one it's GOING TO.
        """
        # this tile should have been assigned as current
        assert self._current_tile is tile
        
        # mark this tile as visited (we are ON it)
        # technically, all tiles in map are visited. This is just a bit more explicit, i guess
        assert not tile.visited
        tile.visited = True

        # since the robot is at the centre of  `tile`,
        # take a reading of the tile type
        # and assign it to the tile
        tile_type = self._robot.tile_type
        assert tile_type not in (TileType.START, TileType.NOGO)
        tile.tile_type = tile_type

        # scan open directions
        open_directions = self._robot.lookaround() # this acts like wait() because it takes some time

        for rel_direction in open_directions:
            assert rel_direction != Direction.SOUTH

            # NOTE: this direction is RELATIVE. we need to convert it to a GLOBAL on.
            global_direction = Direction(rotate(rel_direction, self._robot.heading))

            # get neighbour in that direction
            neighbour = self._map.get_tile_by_direction(tile, global_direction)

            # check if already visited
            if neighbour is not None: # and neighbour.visited: # technically redundant
                continue # next

            neighbour = Tile(tile_point=shift(
                tile.tile_point, global_direction
            ))

            # add neighbour
            self._map.add_tile(neighbour)
            self._map.mark_open(tile, neighbour)

            # move and explore
            # basically:
            #   move_to()
            #   dfs()
            #   move_back()
            # while also handling black tiles (black tile error)
            
            try:
                functions = {
                    Direction.NORTH: (self.advance, self.backtrack),
                    Direction.WEST: (self.advance_left, self.backtrack_left),
                    Direction.EAST: (self.advance_right, self.backtrack_right)
                }

                move_there, move_back = functions[rel_direction]

                move_there()
                self._current_tile = neighbour

                self.dfs(neighbour)
                
                move_back()
                self._current_tile = tile

                wait()

            except BlackTileError:
                # we are back exactly where we started (`tile`)
                self._current_tile = tile

                # TODO: technically we've 'visited' the black tile
                # do we use `visited` anywhere important? if so, might need rethink this
                neighbour.visited = True

                # mark tile as black
                self._map.mark_black(neighbour)

                continue

            # TODO - test this, see if i missed anything


    # movement


    @property
    def _is_black(self) -> bool:
        """Is the tile under the CS black?
        This is just to save me some typing when writing the movements
        """
        return self._robot.tile_type == TileType.NOGO


    # first, the simple ones


    def advance(self):
        """Move forward.
        This is movement #1 in the doc.
        Raise black tile error and return to original position if black tile detected.
        See step_5_a docstring for more info
        """
        initial = TILE_HALF_WIDTH + ADVANCE_MVT_DISTANCE
        self._robot.drive(initial)
        wait()

        if self._is_black:
            self._robot.drive(initial, forward=False)
            raise BlackTileError

        final = TILE_HALF_WIDTH - ADVANCE_MVT_DISTANCE
        self._robot.drive(final)


    def backtrack(self):
        """Move backward.
        This is movement #2 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume both tiles have already been explored
        """
        self._robot.drive(TILE_WIDTH, forward=False)


    # NOTE: all drawings are based on movement #4 (turning right) unless specified otherwise

    # This is the example map we are using:
    '''
    '@' represents the origin of a tile
    +----(0, 0)-----+----(1, 0)-----+
    |               |               |
    |         ↱   +---------+       |
    |    +--@--+  |         @       |
    |    |     |  +---------+       |
    |    |     | ⤴  |               |
    +----|     |----+---------------+
    |    +-----+    |
    |               |
    |       @       |
    |               |
    |               |
    +----(0,-1)-----+
    '''


    def _step_1(self, inverse=False):
        '''
        1) Move backwards
        +----(0, 1)-----+
        |               |
        |               |
        |    +--@--+    |  T            T
        |    |     |    |  |            |
        |    |     |    |  | backwards  |
        +----|     |----+  | distance   |
        |  ↓ +-----+ ↓  |  |            | 1.5x tile height
        |  ↓ +-----+ ↓  |  +            |
        |    |  @  |    |  |            |
        |    |     |    |  | robot height
        |    |     |    |  |            |
        +----+--!--+----+  ⊥            ⊥
        '''

        # estimate
        distance = TILE_WIDTH + TILE_HALF_WIDTH - ROBOT_HEIGHT

        if not inverse:
            self._robot.drive(distance, forward=False)
        else:
            self._robot.drive(distance)


    def _step_2(self, inverse=False):
        '''
        2) Rotate to the left
        +----(0, 1)-----+
        |               |
        |               |
        |       @       |
        |               |
        |               |
        +----(0, 0)-----+
        |               |
        |    +-----+    |
        |   /|  @ /|    |
        |  / |   / |    |
        | /  |  /  |    |
        ++---+-+---+----+
        ⤶     ⤶
        '''

        if not inverse:
            # rotate CLOCKWISE
            self._robot.turn_by(STEP_2_ROTATION)
        else:
            self._robot.turn_by(STEP_2_ROTATION, clockwise=False)


    def _step_3(self, inverse=False):
        '''
        3) Move forward diagonally
        +----(0, 1)-----+----(1, 0)-----+
        |               |               |
        |               |               |
        |         +-----+               |
        |        /     /|               |
        |       /     / |  ↑            |
        +------/-----/--+--↑------------+
        |     +-----+   |  ↑
        |    +-----+    |  ↑
        |   /     /     |  ↑
        |  /     /      |  ↑
        | /     /       |
        ++-----+--------+
        '''

        distance = STEP_3_DISTANCE
        if not inverse:
            self._robot.drive(distance)
        else:
            self._robot.drive(distance, forward=False)


    def _step_4(self, inverse=False):
        '''
        4) Rotate to horizontal
        (closeup diagram)
        We rotate about the turning origin until the robot is aligned
        '%' represents the robot origin
        '$' represents the turning origin (which stays still throughout the rotation)
        +---------(0, 1)----------+
        |                         |
        |                         |
        |   +------------------+  |
        |   |           +----%-|--+
        |===|==========/===$===% /|=======> horizontal centreline
        |   |         /        |/ |
        |   +--------/---------+  |
        |    ↖      /         /   |
        |      ↖   /         /    |
        +---------+---------+-----+

        <----------------->$
        half tile width
            + adjacent
        '''

        # calculate based on previous angle
        angle = 90 - STEP_2_ROTATION

        if not inverse:
            # turn CLOCKWISE
            self._robot.turn_by(angle)
        else:
            self._robot.turn_by(angle, clockwise=False)


    def _step_5(self, inverse=False):
        '''
        5) Move forward to next tile's origin
        '#' represents the left wall
        +----(0, 0)-----+----(1, 0)-----+
        |           → → |               |
        | +---------+ +---------+       |
    # ← | |     @ $ % |         @       |
        | +---------+ +---------+       |
        |           → → |               |
        +---------------+---------------+
        |               |
        |               |
        |       @       |
        |               |
        |               |
        +----(0,-1)-----+
        '''

        # origin is labelled '%' in diagram
        origin_distance_from_left_wall = TILE_HALF_WIDTH + STEP_5_DISTANCE

        # distance between origin and desired origin
        # i.e. distance between '%' and rightmost '@'
        distance = TILE_WIDTH + TILE_HALF_WIDTH - origin_distance_from_left_wall

        if not inverse:
            self._robot.drive(distance)
        else:
            self._robot.drive(distance, forward=False)


    # if we split _step_5 into two forward movements, we get _step_5_a and _step_5_b.
    # use the latter two if we need to check for black tile after step_5_a (or b, if inverse)


    def _step_5_a(self, inverse=False):
        """(when inverse=False) Move just enough to bring the CS onto the tile, to check whether it's black
        +----(0, 0)-----+----(1, 0)-----+
        |               |       |       |
        | +-----+---+-----+     |       |
        | |     |   |   | |     |       |
        | +-----+---+-----+     |       |
        | <--------->   |       |       |
        +------ <---------> ----|-------+
               → → → →        centre

        Relevant closeup of position after calling this function:
        +-------|-+     |       |
        |       | |     |       |
        +-------|-+     |       |
                <=>
            this length
           is in constants.py
        Too small, and you risk not getting into tile (1, 0) at all
        Too big, and you risk getting more than 50% the robot into tile (1, 0), which is not good according to the rules
        """
        origin_distance_from_left_wall = TILE_HALF_WIDTH + STEP_5_DISTANCE
        distance = TILE_WIDTH + STEP_5_A_DISTANCE - origin_distance_from_left_wall

        if not inverse:
            self._robot.drive(distance)
        else:
            self._robot.drive(distance, forward=False)


    def _step_5_b(self, inverse=False):
        """See previous docstrings. (when inverse=False) This drives the remaining distance to next tile's origin"""
        distance = TILE_HALF_WIDTH - STEP_5_A_DISTANCE
        if not inverse:
            self._robot.drive(distance)
        else:
            self._robot.drive(distance, forward=False)


    def advance_right(self):
        """Move forward and to the right.
        This is movement #4 in the doc.
        Raise black tile error and return to original position if black tile detected.
        """
        self._step_1()
        wait()
        self._step_2()
        wait()
        self._step_3()
        wait()

        if self._is_black:
            # do everything backwards to return to initial position
            self._step_3(inverse=True)
            wait()
            self._step_2(inverse=True)
            wait()
            self._step_1(inverse=True)
            raise BlackTileError

        self._step_4()
        wait()

        if self._is_black:
            # do everything backwards to return to initial position
            self._step_4(inverse=True)
            wait()
            self._step_3(inverse=True)
            wait()
            self._step_2(inverse=True)
            wait()
            self._step_1(inverse=True)
            raise BlackTileError

        self._step_5_a()
        wait()

        if self._is_black:
            # do everything backwards to return to initial position
            self._step_5_a(inverse=True)
            wait()
            self._step_4(inverse=True)
            wait()
            self._step_3(inverse=True)
            wait()
            self._step_2(inverse=True)
            wait()
            self._step_1(inverse=True)
            raise BlackTileError

        self._step_5_b()


    def advance_left(self):
        """Move forward and to the left.
        This is movement #3 in the doc.
        Raise black tile error and return to original position if black tile detected.
        """
        self._step_1()
        wait()
        self._step_2(inverse=True)
        wait()
        self._step_3()
        wait()

        if self._is_black:
            self._step_3(inverse=True)
            wait()
            self._step_2()
            wait()
            self._step_1(inverse=True)
            raise BlackTileError

        # TODO: technically, if we rotate and find a black tile, we're not aloud to rotate back out of it.
        self._step_4(inverse=True)
        wait()

        if self._is_black:
            self._step_4()
            wait()
            self._step_3(inverse=True)
            wait()
            self._step_2()
            wait()
            self._step_1(inverse=True)
            raise BlackTileError

        self._step_5_a()
        wait()

        if self._is_black:
            self._step_5_a(inverse=True)
            wait()
            self._step_4()
            wait()
            self._step_3(inverse=True)
            wait()
            self._step_2()
            wait()
            self._step_1(inverse=True)
            raise BlackTileError

        self._step_5_b()


    def backtrack_right(self):
        """Move backward and to the right.
        This is movement #6 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume all three tiles have already been explored
        """
        self._step_5(inverse=True)
        wait()
        self._step_4(inverse=True)
        wait()
        self._step_3(inverse=True)
        wait()
        self._step_2(inverse=True)
        wait()
        self._step_1(inverse=True)


    def backtrack_left(self):
        """Move backward and to the left.
        This is movement #5 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume all three tiles have already been explored
        """
        self._step_5(inverse=True)
        wait()
        self._step_4()
        wait()
        self._step_3(inverse=True)
        wait()
        self._step_2()
        wait()
        self._step_1(inverse=True)