#!/usr/bin/env python3

import math
import time
from typing import Optional

from shapely.geometry import Point

from constants import *
from utils import *

from robot import Robot
from map import Tile, Wall, Map
from indicate import *

__all__ = ['Solver']


class NogoTileError(Exception):
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
            map_point=Point(0, 0), # define the global origin to be at the start tile origin
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
        next_tile = Tile(map_point=Point(0, 1))
        self._map.add_tile(next_tile)

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
        rel_directions = self._robot.lookaround() # this acts like wait() because it takes some time
        global_directions = [Direction.from_heading(rel_direction.rotate(self._robot.heading)) for rel_direction in rel_directions]

        # update map with walls
        for direction in Direction:
            if direction not in global_directions:
                # there is a wall
                self._map.create_wall_by_direction(tile, direction)

        for rel_direction, global_direction in zip(rel_directions, global_directions):
            assert rel_direction != Direction.SOUTH

            # get neighbour in that direction
            neighbour = self._map.get_tile_by_direction(tile, global_direction, require_open=False)

            # check if already visited
            if neighbour is not None: # and neighbour.visited: # technically redundant because we only add tiles to the map if the tile has been visited
                assert neighbour.visited

                # remember to mark that tile as 'open' from this tile
                self._map.mark_open(tile, neighbour)

                continue # next

            neighbour = self._map.create_tile_by_direction(tile, global_direction)
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

            except NogoTileError:
                # we are back exactly where we started (`tile`)
                self._current_tile = tile

                # TODO: technically we've 'visited' the black tile
                # do we use `visited` anywhere important? if so, might need rethink this
                neighbour.visited = True

                # mark tile as black
                self._map.mark_nogo(neighbour)

                continue


    # movement
    # each movement should assume the robot origin is EXACTLY
    # on the current tile's origin
    # and each movement should (try to) move the robot
    # so that the origin is EXACTLY on the target tile's origin

    
    @property
    def _is_nogo(self) -> bool:
        """Is the tile under the CS black?
        This is just to save me some typing when writing the movements
        """
        return self._robot.tile_type == TileType.NOGO


    def drive_forward_until_not_moving(self, speed: Speed = DEFAULT_STRAIGHT_SPEED):
        """Keep driving forward until the US readings are stable (no changes), plus a little extra distance (DRIVE_WALL_EXTRA_DISTANCE).
        Use this to drive forward until hit wall
        NOTE: US must be facing a wall (and the wall must be within range) for this to work
        NOTE: Remember to calibrate position after this!
        """
        self._robot.us_turn_to(Direction.NORTH)

        last_reading = self._robot.us_distance

        # start driving
        with self._robot.driving(speed=speed) as distance:
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

        # extra distance (robot is already at the wall, this just hopefully corrects the heading by ramming against the wall)
        # no need to do wait() because we are driving in one direction near-continuously
        self._robot.drive(DRIVE_WALL_EXTRA_DISTANCE, speed=speed)

        # right now the odometry is all messed up
        # because the wheels have been spinning while slipping, and the robot not moving
        # so the caller should calibrate to the expected origin position


    def drive_backward_until_not_moving(self, speed: Speed = DEFAULT_STRAIGHT_SPEED):
        """Keep driving backwards (and rotating slightly) until both TS activate
        Use this to drive backward until hit wall
        NOTE: Remember to calibrate position after this!
        """
        with self._robot.driving(forward=False, speed=speed) as distance:
            while True:
                if self._robot.touching_rel_directions or distance.value >= DRIVE_WALL_MAX_DISTANCE:
                    # break if at least one is touching
                    break
                wait(DRIVE_WALL_POLL_INTERVAL_MS)

        if len(self._robot.touching_rel_directions) == 2:
            # fully aligned
            self._robot.drive(DRIVE_WALL_EXTRA_DISTANCE, forward=False, speed=speed)
            return

        # these are too unimportant to be constants (?)
        small_angle = 3
        small_distance = 5 # mm
        max_corrections = 5

        corrections = 0
        while True:
            if corrections == max_corrections:
                # could be stuck, just return at this point
                break
            corrections += 1

            if len(self._robot.touching_rel_directions) == 2:
                return # extra distance probably unnecessary

            if self._robot.is_touching_rel_direction(Direction.EAST):
                # don't use given speed here, as deltas are very small
                self._robot.turn_by(small_angle, clockwise=False)
                self._robot.drive(small_distance, forward=False)

            elif self._robot.is_touching_rel_direction(Direction.WEST):
                self._robot.turn_by(small_angle)
                self._robot.drive(small_distance, forward=False)


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
                raise NogoTileError

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
            if self._map.get_wall_by_direction(current_tile, global_direction) is None:
                self._robot.us_turn_to(Direction.NORTH)
                if self._robot.is_open:
                    return

            # here, we have confirmed there is a wall directly in front of the robot
            # we can move forward and calibrate
            wait()

            # move forward until we hit the wall
            self.drive_forward_until_not_moving()

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
        # NO CALIBRATION
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
        """
        1) Move backwards
        +----(0, 1)-----+
        |               |
        |               |
        |    +--@--+    |  T
        |    |     |    |  | backwards
        |    |     | ↓  |  | distance
        +---+|----+|-↓--+  ⊥
        |   |+----|+ ↓  |
        |   |     |  ↓  |
        |   |     |  ↓  |
        |   + ----+     |
        |               |
        +----+--!--+----+
        """
        if not inverse:
            self._robot.drive(TILE_HALF_WIDTH, forward=False)
        else:
            self._robot.drive(TILE_HALF_WIDTH)


    def _turn_step_2(self, inverse=False):
        """
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
        """
        if not inverse:
            self._robot.turn_by(STEP_2_ROTATION)
        else:
            self._robot.turn_by(STEP_2_ROTATION, clockwise=False)


    def _turn_step_3(self, inverse=False):
        """
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
        """
        if not inverse:
            self._robot.drive(STEP_3_DISTANCE)
        else:
            self._robot.drive(STEP_3_DISTANCE, forward=False)


    def _turn_step_4(self, inverse=False):
        """
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
        """
        if not inverse:
            self._robot.turn_by(STEP_4_ROTATION)
        else:
            self._robot.turn_by(STEP_4_ROTATION, clockwise=False)


    def _turn_step_5_no_calibrate(self, inverse=False, raise_if_nogo=True):
        """Internal helper for calibrate=False case
        No inverse:

        +----(0, 0)-----+----(1, 0)-----+
        |       |       |               |
        | +-----+---+-----+             |
        | |     |   |   | |             |
        | +-----+---+-----+             |
        |      →|→ → →  |               |
        +-------|-------+---------------+
              centre
                <--->
            STEP_5_DISTANCE
                    <----->
                STEP_5_A_DISTANCE
                        <=>
                       buffer
        If normal:
        +----(0, 0)-----+----(1, 0)-----+
        |               |               |
        |       +-----+---+-----+       |
        |       |     | | |     @       |
        |       +-----+---+-----+       |
        |            → → → →            |
        +---------------+---------------+
                          <----->
                        STEP_5_B_DISTANCE
                        <=>
                       buffer

        If black:
        +----(0, 0)-----+----(1, 0)-----+
        |               |               |
        | +-----+---+-----+             |
        | |     |   |   | |             |
        | +-----+---+-----+             |
        |      ← ← ← ←  |               |
        +---------------+---------------+
                    <----->
                STEP_5_A_DISTANCE

        Inverse:

        +----(0, 0)-----+----(1, 0)-----+
        |               |               |
        | +---------+ +---------+       |
        | |         | | |       @       |
        | +---------+ +---------+       |
        |         ← ← ← ←               |
        +---------------+---------------+
                    <-----><---->
        STEP_5_A_DISTANCE   STEP_5_B_DISTANCE
        """
        if not inverse:
            self._robot.drive(STEP_5_A_DISTANCE)

            if raise_if_nogo and self._is_nogo:
                wait()
                self._robot.drive(STEP_5_A_DISTANCE, forward=False)
                raise NogoTileError

            self._robot.drive(STEP_5_B_DISTANCE)

        else:
            distance = STEP_5_A_DISTANCE + STEP_5_B_DISTANCE
            self._robot.drive(distance, forward=False)


    def _turn_step_5(self, inverse=False, raise_if_nogo=True, calibrate=True):
        """
        5) Move forward to next tile's origin
        +----(0, 0)-----+----(1, 0)-----+
        |           → → |               |
        | +---------+ +---------+       |
        | |     @ $ % |         @       |
        | +---------+ +---------+       |
        |           → → |               |
        +---------------+---------------+
        """
        if not calibrate:
            self._turn_step_5_no_calibrate(inverse, raise_if_nogo)

        else:
            '''
            No inverse:

            +----(0, 0)-----+----(1, 0)-----+
            |               |               |
            +-+-------+-+   |               |
            | |       | |   |               |
            +-+-------+-+   |               |
            |  ← ← ← ←      |               |
            +---------------+---------------+
            +----(0, 0)-----+----(1, 0)-----+
            |               |               |
            +-------+-+-------+             |
            |       | |     | |             |
            +-------+-+-------+             |
            |       → → → → |               |
            +---------------+---------------+
                    <------><=>
        tile width - height   buffer
            If normal:
            +----(0, 0)-----+----(1, 0)-----+
            |               |               |
            |       +-----+---+-----+       |
            |       |     | | |     @       |
            |       +-----+---+-----+       |
            |            → → → →            |
            +---------------+---------------+
                              <----->
                            STEP_5_B_DISTANCE
            If black:
            +----(0, 0)-----+----(1, 0)-----+
            |               |               |
            | +-----+---+-----+             |
            | |     |   |   | |             |
            | +-----+---+-----+             |
            |      ← ← ← ←  |               |
            +---------------+---------------+
                        <----->
                    STEP_5_A_DISTANCE
            
            Inverse:

            +----(0, 0)-----+----(1, 0)-----+
            |               |               |
            +---------+   +---------+       |
            |         |   | |       @       |
            +---------+   +---------+       |
            |        ← ← ← ←                |
            +---------------+---------------+
            +----(0, 0)-----+----(1, 0)-----+
            |               |               |
            +-+-------+-+   |               |
            | |       | |   |               |
            +-+-------+-+   |               |
            |  → → → →      |               |
            +---------------+---------------+
            <=>
            ???
            '''
            # calibrate only if there is a wall on the left/right (depending on `inverse`)
            global_direction = Direction.from_heading(self._robot.heading)
            wall_direction = global_direction.reverse() # relative 'south'

            if not inverse:
                current_tile = self._current_tile
            else:
                current_tile = self._map.get_tile_by_direction(
                    self._current_tile,
                    wall_direction,
                    require_open=True
                )
                assert current_tile is not None

            if self._map.get_wall_by_direction(current_tile, wall_direction) is None:
                # cannot calibrate, as no wall
                self._turn_step_5_no_calibrate(inverse, raise_if_nogo)
                return

            if not inverse:
                # move to back wall
                self.drive_backward_until_not_moving()
                wait()

                # move to the connection between the tiles
                self._robot.drive(TILE_WIDTH - ROBOT_HEIGHT)
                # calibrate
                self._robot.origin = shift(current_tile.origin, global_direction, TILE_HALF_WIDTH)

                # move remaining buffer distance
                self._robot.drive(NOGO_DETECTION_BUFFER)

                if raise_if_nogo and self._is_nogo:
                    wait()
                    self._robot.drive(STEP_5_A_DISTANCE, forward=False)
                    raise NogoTileError

                # move to final
                self._robot.drive(STEP_5_B_DISTANCE)

            else:
                # no need to consider black tiles
                # in backtrack mode

                # move to back wall
                self.drive_backward_until_not_moving()
                wait()

                # move forward to initial
                distance = TILE_HALF_WIDTH - (ROBOT_HEIGHT - STEP_5_DISTANCE)
                self._robot.drive(distance)


    def advance_right(self, handle_nogo_tiles: bool = True, calibrate: bool = True):
        """Move forward and to the right.
        This is movement #4 in the doc.
        Raise black tile error and return to original position if black tile detected.
        """
        self._turn_step_1()
        wait()
        self._turn_step_2()
        wait()
        self._turn_step_3()
        wait()
        # TODO: technically, if we rotate and find a black tile, we're not aloud to rotate back out of it.
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
            raise NogoTileError

        try:
            self._turn_step_5(raise_if_nogo=handle_nogo_tiles, calibrate=calibrate)
        except NogoTileError:
            # we are back to previous position
            wait()
            indicate_nogo_tile()
            # do everything backwards to return to initial position
            self._turn_step_4(inverse=True)
            wait()
            self._turn_step_3(inverse=True)
            wait()
            self._turn_step_2(inverse=True)
            wait()
            self._turn_step_1(inverse=True)
            raise # reraise


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
            raise NogoTileError

        try:
            self._turn_step_5(raise_if_nogo=handle_nogo_tiles, calibrate=calibrate)
        except NogoTileError:
            wait()
            indicate_nogo_tile()
            self._turn_step_4()
            wait()
            self._turn_step_3(inverse=True)
            wait()
            self._turn_step_2()
            wait()
            self._turn_step_1(inverse=True)


    def backtrack_right(self, calibrate: bool = True):
        """Move backward and to the right.
        This is movement #6 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume all three tiles have already been explored
        """
        self._turn_step_5(inverse=True, raise_if_nogo=False, calibrate=calibrate)
        wait()
        self._turn_step_4(inverse=True)
        wait()
        self._turn_step_3(inverse=True)
        wait()
        self._turn_step_2(inverse=True)
        wait()
        self._turn_step_1(inverse=True)


    def backtrack_left(self, calibrate: bool = True):
        """Move backward and to the left.
        This is movement #5 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume all three tiles have already been explored
        """
        self._turn_step_5(inverse=True, raise_if_nogo=False, calibrate=calibrate)
        wait()
        self._turn_step_4()
        wait()
        self._turn_step_3(inverse=True)
        wait()
        self._turn_step_2()
        wait()
        self._turn_step_1(inverse=True)




