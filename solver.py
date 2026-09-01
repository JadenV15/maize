#!/usr/bin/env python3

import math
import time
from typing import Optional

from shapely.geometry import Point

from constants import *
from utils import *

from robot import Robot
from map import Tile, Edge, Map
from indicate import *

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


    def solve(self, test_initial_only: bool = False):
        """Main entry point"""
        # assert we are starting fresh
        assert self._map.is_empty
        try:
            self._current_tile
        except AttributeError: pass
        else:
            raise Exception

        indicate_start()

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

        # add current tile
        start_tile = Tile(
            tile_point=Point(0, 0), # define the global origin to be at the start tile origin
            tile_type=TileType.START, # this is a Start tile
            visited=True # we have 'visited' the start tile
        )
        self._map.add_tile(start_tile)

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
        self._map.add_tile(next_tile)
        self._map.mark_open(start_tile, next_tile)

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

        if test_initial_only:
            # Exit, no dfs
            return
        
        wait()

        # assign current Tile
        self._current_tile = next_tile # type: Tile

        # start exploring from tile (0,1)
        self.dfs(next_tile)

        wait()

        # finally, move back to the start tile and align against back wall
        self._robot.drive(delta, forward=False)

        indicate_finish()
        indicate_final_counts(self._map.statistics)


    # dfs


    def dfs(self, tile: Tile):
        """Entry point for dfs.
        For the first call, the robot is expected to be in the tile north of the start tile,
        origin at the tile origin,
        facing global north.
        This func is called recursively.
        NOTE: <tile> always refers to the tile the robot is ON, NOT the one it's GOING TO.
        """
        # note: the assumption is that
        # right now, the robot's origin is EXACTLY (i.e. has been calibrated)
        # on the current tile's origin

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

        # announce if applicable
        if tile_type == TileType.HARMED_VICTIM:
            indicate_harmed_victim()
        elif tile_type == TileType.UNHARMED_VICTIM:
            indicate_unharmed_victim()

        # scan open directions
        open_directions = self._robot.lookaround() # this acts like wait() because it takes some time

        for rel_direction in open_directions:
            assert rel_direction != Direction.SOUTH

            # NOTE: this direction is RELATIVE. we need to convert it to a GLOBAL on.
            global_direction = Direction.from_heading(rel_direction.rotate(self._robot.heading))

            # get neighbour in that direction
            neighbour = self._map.get_tile_by_direction(tile, global_direction, require_open=False)

            # check if already visited
            if neighbour is not None: # and neighbour.visited: # technically redundant because we only add tiles to the map if the tile has been visited
                assert neighbour.visited

                # remember to mark that tile as 'open' from this tile
                self._map.mark_open(tile, neighbour)

                continue # next

            neighbour = self._map.create_tile_by_direction(tile, global_direction, mark_open=True)

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
                self._map.mark_nogo(neighbour)

                continue

            # TODO - test this, see if i missed anything


    # movement
    # each movement should assume the robot origin is EXACTLY
    # on the current tile's origin
    # and each movement should (try to) move the robot
    # so that the origin is EXACTLY on the target tile's origin
    # (^^^ TODO - calibration)

    
    @property
    def _is_nogo(self) -> bool:
        """Is the tile under the CS black?
        This is just to save me some typing when writing the movements
        """
        return self._robot.tile_type == TileType.NOGO


    # i actually hate this style of indentation
    def drive_until_us_stable(self,
                              min_distance: Optional[Numeric] = None,
                              calibrate_origin: Optional[Point] = None,
                              forward: bool = True,
                              speed: Speed = DEFAULT_STRAIGHT_SPEED):
        """Keep driving until the US readings are stable (no changes), plus a little extra distance (DRIVE_WALL_EXTRA_DISTANCE). Then calibrate the robot origin to <calibrate_origin>
        Use this to drive until hit wall
        NOTE: US must be facing a wall (and the wall must be within range) for this to work
        """
        # TODO: for now don't call this with min_distance, because this could mask bugs

        last_reading = self._robot.us_distance

        # start driving
        with self._robot.driving(forward, speed) as distance:
            wait(DRIVE_WALL_POLL_INTERVAL_MS) # TODO - just in case the motors don't start immediately

            while True:
                wait(DRIVE_WALL_POLL_INTERVAL_MS)

                # check if distance limit reached
                if distance.value >= DRIVE_WALL_MAX_DISTANCE:
                    break

                # check if US reading is stable
                current_reading = self._robot.us_distance
                if math.isclose(last_reading, current_reading, abs_tol=0.1):
                    break
                last_reading = current_reading

        if min_distance is not None:
            delta = min_distance - distance.value
            if delta > 0:
                # we are less than min_distance
                self._robot.drive(delta, forward, speed)

        # extra distance (robot is already at the wall, this just hopefully corrects the heading by ramming against the wall)
        # no need to do wait() because we are driving in one direction near-continuously
        self._robot.drive(DRIVE_WALL_EXTRA_DISTANCE, forward, speed)

        # right now the odometry is all messed up
        # because the wheels have been spinning while slipping, and the robot not moving
        # so we calibrate to the expected origin position
        # if provided
        if calibrate_origin is not None:
            self._robot.origin = calibrate_origin


    # first, the simple ones


    def advance(self, handle_nogo_tiles: bool = True, calibrate: bool = True):
        """Move forward.
        This is movement #1 in the doc.
        Raise black tile error and return to original position if black tile detected.
        See step_5_a docstring for more info
        Diagram:
        '@' represents tile origin (and here, also robot origin)
        +--wall----+
        |          |
        | +--@--+  | current_tile
        | |     |  |
        +-|-----|--+ ↑
        | |_____|  | ↑
        | +--@--+  | ↑ self._current_tile
        | |     |  | ↑
        +-|-----|--+ ↑
        | |_____|  |
        |          |
        |          |
        +----------+
        If 'wall' exists, move up to it then back, to calibrate
        TODO: is there enough time to calibrate?
        """
        if handle_nogo_tiles:
            self._robot.drive(ADVANCE_A_DISTANCE)
            wait()

            if self._is_nogo:
                indicate_nogo_tile()
                self._robot.drive(ADVANCE_A_DISTANCE, forward=False)
                raise BlackTileError

            self._robot.drive(ADVANCE_B_DISTANCE)

        else:
            self._robot.drive(TILE_WIDTH)

        if calibrate:
            # TODO - THIS IS EXPERIMENTAL

            global_direction = Direction.from_heading(self._robot.heading)
            current_tile = self._map.get_tile_by_direction(
                self._current_tile,
                global_direction,
                require_open=True
            )

            # current_tile should have been created in dfs()
            assert current_tile is not None

            # calibrate only if there is a wall

            if self._map.is_open_direction(current_tile, global_direction):
                return

            self._robot.us_turn_to(Direction.NORTH) # which would be along global_direction
            if self._robot.is_open:
                return

            # here, we have confirmed there is a wall directly in front of the robot
            # we can move forward and calibrate
            wait()

            # since the US is already pointing relative North,
            # we can call drive_until_us_stable
            # and move forward until we hit the wall
            self.drive_until_us_stable(calibrate_origin=shift(
                current_tile.origin, global_direction, TILE_HALF_WIDTH
            ))
            wait()

            # now move back
            self._robot.drive(TILE_HALF_WIDTH, forward=False)

            # calibrate origin to tile center
            self._robot.origin = current_tile.origin


    def backtrack(self):
        """Move backward.
        This is movement #2 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume both tiles have already been explored
        """
        # NO CALIBRATION - TODO?

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


    def _turn_step_1(self, inverse=False):
        '''
        1) Move backwards
        +----(0, 1)-----+
        |               |
        |               |
        |    +--@--+    |  T
        |    |     |    |  | backwards
        |    |     | ↓  |  | distance (half tile width)
        +---+|----+|-↓--+  ⊥
        |   |+----|+ ↓  |
        |   |     |  ↓  |
        |   |     |  ↓  |
        |   + ----+     |
        |               |
        +----+--!--+----+
        '''
        if not inverse:
            self._robot.drive(TILE_HALF_WIDTH, forward=False)
        else:
            self._robot.drive(TILE_HALF_WIDTH)


    def _turn_step_2(self, inverse=False):
        '''
        2) Rotate to the left
        +----(0, 1)-----+
        |               |
        |               |
        |       @       |
        |               |
        |               |
        +----+-----+----+
        |   /|    /|    |
        |  / |   / |    |
        | /  |  /  |    |
        |+---+-+---+    |
        |  ⤶     ⤶      |
        +---------------+
        '''
        if not inverse:
            self._robot.turn_by(STEP_2_ROTATION)
        else:
            self._robot.turn_by(STEP_2_ROTATION, clockwise=False)


    def _turn_step_3(self, inverse=False):
        '''
        3) Move forward diagonally
        +----(0, 1)-----+----(1, 0)-----+
        |               |               |
        |               |               |
        |         +-----+               |
        |        /     /|               |
        |       /     / |  ↑            |
        +-----+/----+/--+--↑------------+
        |    /+----/+   |  ↑
        |   /     /     |  ↑
        |  /     /      |  ↑
        | +-----+       |  ↑
        |               |
        +---------------+
        '''
        # TODO: to avoid snagging the US on the sensor
        # we rotate it rel North
        # but is this necessary?
        self._robot.us_turn_to(Direction.NORTH)
        if not inverse:
            self._robot.drive(STEP_3_DISTANCE)
        else:
            self._robot.drive(STEP_3_DISTANCE, forward=False)


    def _turn_step_4(self, inverse=False):
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
        if not inverse:
            self._robot.turn_by(STEP_4_ROTATION)
        else:
            self._robot.turn_by(STEP_4_ROTATION, clockwise=False)


    def _turn_step_5(self, inverse=False):
        '''
        5) Move forward to next tile's origin
        '#' represents the left wall
        +----(0, 0)-----+----(1, 0)-----+
        |           → → |               |
        | +---------+ +---------+       |
        | |     @ $ % |         @       |
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
        if not inverse:
            self._robot.drive(STEP_5_DISTANCE)
        else:
            self._robot.drive(STEP_5_DISTANCE, forward=False)


    # if we split _step_5 into two forward movements, we get _step_5_a and _step_5_b.
    # use the latter two if we need to check for black tile after step_5_a (or b, if inverse)


    def _turn_step_5_a(self, inverse=False):
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
               buffer

        Distance too small, and you risk not getting into tile (1, 0) at all
        Too big, and you risk getting more than 50% the robot into tile (1, 0), which is not good according to the rules
        """
        if not inverse:
            self._robot.drive(STEP_5_A_DISTANCE)
        else:
            self._robot.drive(STEP_5_A_DISTANCE, forward=False)
        

    def _turn_step_5_a_with_calibrate(self, inverse=False):
        """This is basically step 5a but with calibration.
        If there is a wall behind, back into that wall, then move forward into the next tile.
        If there is no wall, do step_5_a as usual.
        +----(0, 0)-----+----(1, 0)-----+
        |               |               |
        +-----+---+-----+               |
        |     |   |     |               |
        +-----+---+-----+               |
        |   ← ← ← ← ←   |               |
        +---------------+---------------+
        +----(0, 0)-----+----(1, 0)-----+
        |               |               |
        +-------+-+-------+             |
        |       | |     | |             |
        +-------+-+-------+             |
        |       → → → → |               |
        +---------------+---------------+
                        <=>
                       buffer
        """
        # TODO



    def _turn_step_5_b(self, inverse=False):
        """See previous docstrings. (when inverse=False) This drives the remaining distance to next tile's origin"""
        if not inverse:
            self._robot.drive(STEP_5_B_DISTANCE)
        else:
            self._robot.drive(STEP_5_B_DISTANCE, forward=False)


    def advance_right(self, handle_nogo_tiles: bool = True, calibrate: bool = True):
        """Move forward and to the right.
        This is movement #4 in the doc.
        Raise black tile error and return to original position if black tile detected.
        """
        # TODO: handle calibration
        self._turn_step_1()
        wait()
        self._turn_step_2()
        wait()
        self._turn_step_3()
        wait()

        if handle_nogo_tiles and self._is_nogo:
            indicate_nogo_tile()
            # do everything backwards to return to initial position
            self._turn_step_3(inverse=True)
            wait()
            self._turn_step_2(inverse=True)
            wait()
            self._turn_step_1(inverse=True)
            raise BlackTileError

        self._turn_step_4()
        wait()

        if handle_nogo_tiles and self._is_nogo:
            indicate_nogo_tile()
            # do everything backwards to return to initial position
            self._turn_step_4(inverse=True)
            wait()
            self._turn_step_3(inverse=True)
            wait()
            self._turn_step_2(inverse=True)
            wait()
            self._turn_step_1(inverse=True)
            raise BlackTileError

        if calibrate:
            self._turn_step_5_a_with_calibrate()
        else:
            self._turn_step_5_a()
        wait()

        if handle_nogo_tiles and self._is_nogo:
            indicate_nogo_tile()
            # do everything backwards to return to initial position
            if calibrate:
                self._turn_step_5_a_with_calibrate(inverse=True)
            else:
                self._turn_step_5_a(inverse=True)
            wait()
            self._turn_step_4(inverse=True)
            wait()
            self._turn_step_3(inverse=True)
            wait()
            self._turn_step_2(inverse=True)
            wait()
            self._turn_step_1(inverse=True)
            raise BlackTileError

        self._turn_step_5_b()

        if handle_nogo_tiles and self._is_nogo:
            wait()
            indicate_nogo_tile()
            # do everything backwards to return to initial position
            self._turn_step_5_b(inverse=True)
            wait()
            if calibrate:
                self._turn_step_5_a_with_calibrate(inverse=True)
            else:
                self._turn_step_5_a(inverse=True)
            wait()
            self._turn_step_4(inverse=True)
            wait()
            self._turn_step_3(inverse=True)
            wait()
            self._turn_step_2(inverse=True)
            wait()
            self._turn_step_1(inverse=True)
            raise BlackTileError


    def advance_left(self, handle_nogo_tiles: bool = True, calibrate: bool = True):
        """Move forward and to the left.
        This is movement #3 in the doc.
        Raise black tile error and return to original position if black tile detected.
        """
        # TODO: handle calibration
        self._turn_step_1()
        wait()
        self._turn_step_2(inverse=True)
        wait()
        self._turn_step_3()
        wait()

        if handle_nogo_tiles and self._is_nogo:
            indicate_nogo_tile()
            self._turn_step_3(inverse=True)
            wait()
            self._turn_step_2()
            wait()
            self._turn_step_1(inverse=True)
            raise BlackTileError

        # TODO: technically, if we rotate and find a black tile, we're not aloud to rotate back out of it.
        self._turn_step_4(inverse=True)
        wait()

        if handle_nogo_tiles and self._is_nogo:
            indicate_nogo_tile()
            self._turn_step_4()
            wait()
            self._turn_step_3(inverse=True)
            wait()
            self._turn_step_2()
            wait()
            self._turn_step_1(inverse=True)
            raise BlackTileError

        if calibrate:
            self._turn_step_5_a_with_calibrate(inverse=True)
        else:
            self._turn_step_5_a(inverse=True)
        wait()

        if handle_nogo_tiles and self._is_nogo:
            indicate_nogo_tile()
            if calibrate:
                self._turn_step_5_a_with_calibrate(inverse=True)
            else:
                self._turn_step_5_a(inverse=True)
            wait()
            self._turn_step_4()
            wait()
            self._turn_step_3(inverse=True)
            wait()
            self._turn_step_2()
            wait()
            self._turn_step_1(inverse=True)
            raise BlackTileError

        self._turn_step_5_b()

        if handle_nogo_tiles and self._is_nogo:
            wait()
            indicate_nogo_tile()
            # do everything backwards to return to initial position
            self._turn_step_5_b(inverse=True)
            wait()
            if calibrate:
                self._turn_step_5_a_with_calibrate(inverse=True)
            else:
                self._turn_step_5_a(inverse=True)
            wait()
            self._turn_step_4()
            wait()
            self._turn_step_3(inverse=True)
            wait()
            self._turn_step_2()
            wait()
            self._turn_step_1(inverse=True)
            raise BlackTileError


    def backtrack_right(self):
        """Move backward and to the right.
        This is movement #6 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume all three tiles have already been explored
        """
        self._turn_step_5(inverse=True)
        wait()
        self._turn_step_4(inverse=True)
        wait()
        self._turn_step_3(inverse=True)
        wait()
        self._turn_step_2(inverse=True)
        wait()
        self._turn_step_1(inverse=True)


    def backtrack_left(self):
        """Move backward and to the left.
        This is movement #5 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume all three tiles have already been explored
        """
        self._turn_step_5(inverse=True)
        wait()
        self._turn_step_4()
        wait()
        self._turn_step_3(inverse=True)
        wait()
        self._turn_step_2()
        wait()
        self._turn_step_1(inverse=True)