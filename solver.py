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

        self._solve_started_at = None # type: Optional[float]


    @property
    def _elapsed_seconds(self) -> float:
        if self._solve_started_at is None:
            return 0.0
        return time.monotonic() - self._solve_started_at


    def _debug(self, message: str):
        elapsed = self._elapsed_seconds
        print('[SOLVER] [{:.1f}s] {}'.format(elapsed, message))


    def solve(self, test_initial_only: bool = False):
        """Main entry point"""
        self._solve_started_at = time.monotonic()
        self._debug('solve start: test_initial_only={}'.format(test_initial_only))
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
        self._debug('start tile added at {}'.format(start_tile.map_point))

        # calibrate the robot origin to the actual origin
        # the below var represents '@' in the diagram. We find it by shifting up from '!' by <robot height>
        origin = shift(
            shift(start_tile.origin, Direction.SOUTH, TILE_HALF_WIDTH), # '!' in the above diagram
            Direction.NORTH,
            ROBOT_HEIGHT
        )
        self._robot.origin = origin
        self._robot.heading = Direction.NORTH
        self._debug('robot calibrated at start: origin={}, heading={:.1f}'.format(origin, self._robot.heading))

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
        self._debug('initial next tile added at {}'.format(next_tile.map_point))

        # move
        delta = origin.distance(next_tile.origin) # distance to move forward by
        self._robot.drive(delta)
        self._debug('initial advance complete: distance={:.1f}mm'.format(delta))
        #next_tile.visited = True
        # ^^^ don't do this, as dfs() will mark it as visited

        # at this point we are set up

        if test_initial_only:
            # Exit, no dfs
            return
        
        wait()

        # assign current Tile
        self._current_tile = next_tile # type: Tile

        # align US sensor
        self._robot.us_calibrate_to_north()

        # start exploring from tile (0,1)
        self.dfs(next_tile)
        self._debug('DFS complete; returning to start')

        wait()

        # finally, move back to the start tile and align against back wall
        self._robot.drive(delta, forward=False)

        indicate_finish()
        indicate_final_counts(self._map.statistics)
        self._debug('solve complete')
        self._solve_started_at = None


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
        self._debug('DFS enter tile={} origin={}'.format(tile.map_point, tile.origin))
        
        # mark this tile as visited (we are ON it)
        # technically, all tiles in map are visited. This is just a bit more explicit, i guess
        assert not tile.visited
        tile.visited = True

        # since the robot is at the centre of  `tile`,
        # we take a reading of the tile type
        # and assign it to the tile
        tile_type = self.get_smart_tile_type() # use smart
        assert tile_type not in (TileType.START, TileType.NOGO)
        tile.tile_type = tile_type
        self._debug('tile {} type={}'.format(tile.map_point, tile_type))

        # announce if applicable
        if tile_type == TileType.HARMED_VICTIM:
            indicate_harmed_victim()
        elif tile_type == TileType.UNHARMED_VICTIM:
            indicate_unharmed_victim()

        # scan open directions
        rel_directions = self._robot.lookaround() # this acts like wait() because it takes some time
        # do north first, because forward/backward are most reliable, and we would rather the robot fail later than earlier
        rel_directions = [Direction.NORTH] + [d for d in rel_directions if d != Direction.NORTH] if Direction.NORTH in rel_directions else rel_directions

        # convert to global
        global_directions = [Direction.from_heading(rel_direction.rotate(self._robot.heading)) for rel_direction in rel_directions]
        global_south = Direction.from_heading(Direction.SOUTH.rotate(self._robot.heading))

        self._debug('scan: relative_open={}, global_open={}, ignored_south={}'.format(
            rel_directions, global_directions, global_south
        ))

        # update map with walls
        for direction in Direction:
            if direction == global_south:
                # ignore south, as US cannot turn south
                continue
            if direction not in global_directions:
                # there is a wall
                self._map.create_wall_by_direction(tile, direction)
                self._debug('wall recorded: tile={}, direction={}'.format(tile.map_point, direction))

        for rel_direction, global_direction in zip(rel_directions, global_directions):
            assert rel_direction != Direction.SOUTH

            # get neighbour in that direction
            neighbour = self._map.get_tile_by_direction(tile, global_direction, require_open=False)
            self._debug('edge considered: {} -> {} via {}'.format(tile.map_point, global_direction, neighbour.map_point if neighbour else 'new'))

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

                self._debug('move to {} from {}: relative={}, global={}'.format(
                    neighbour.map_point, tile.map_point, rel_direction, global_direction
                ))

                move_there()
                self._current_tile = neighbour

                self.dfs(neighbour)
                
                move_back()
                self._current_tile = tile

                wait()
                self._debug('returned to tile {}'.format(tile.map_point))

            except NogoTileError:
                # we are back exactly where we started (`tile`)
                self._current_tile = tile

                # TODO: technically we've 'visited' the black tile
                # do we use `visited` anywhere important? if so, might need rethink this
                neighbour.visited = True

                # mark tile as black
                self._map.mark_nogo(neighbour)
                self._debug('NOGO recorded at {}'.format(neighbour.map_point))

                continue


    # robot utils


    def get_smart_tile_type(self) -> TileType:
        """Move back and forth until a proper tiletype is detected, then return to original position.
        Robot is assumed to be centered in the tile
        """
        current_tile_type = self._robot.tile_type
        if current_tile_type is not None:
            return current_tile_type

        small_distance = 5 # mm # TODO: constant?
        max_distance = 25 # max abs distance from initial origin

        abs_distance = 0 # total abs distance from initial origin
        initial_origin = self._robot.origin

        while True:
            if abs_distance + small_distance >= max_distance:
                # driving back again would exceed max
                # try again, but moving forward

                # first move back to origin
                self._robot.drive(abs_distance)
                self._robot.origin = initial_origin
                # try again at origin just for luck
                current_tile_type = self._robot.tile_type
                if current_tile_type is not None:
                    return current_tile_type
                
                abs_distance = 0
                while True:
                    if abs_distance + small_distance >= max_distance:
                        self._robot.drive(abs_distance, forward=False)
                        self._robot.origin = initial_origin
                        # one final try just for luck
                        current_tile_type = self._robot.tile_type
                        if current_tile_type is not None:
                            return current_tile_type

                        else:
                            return TileType.NORMAL # pretend its normal
                        
                    self._robot.drive(small_distance)
                    abs_distance += small_distance

                    current_tile_type = self._robot.tile_type
                    if current_tile_type is not None:
                        wait()
                        self._robot.drive(abs_distance, forward=False)
                        self._robot.origin = initial_origin
                        return current_tile_type
            
            self._robot.drive(small_distance, forward=False)
            abs_distance += small_distance

            current_tile_type = self._robot.tile_type
            if current_tile_type is not None:
                wait()
                self._robot.drive(abs_distance)
                # calibrate
                self._robot.origin = initial_origin
                return current_tile_type


    def drive_forward_until_not_moving(self, speed: Speed = DEFAULT_STRAIGHT_SPEED):
        """Keep driving forward until the US readings are stable (no changes), plus a little extra distance (DRIVE_WALL_EXTRA_DISTANCE).
        Use this to drive forward until hit wall
        NOTE: US must be facing a wall (and the wall must be within range) for this to work
        NOTE: Remember to calibrate position after this!
        """
        self._debug('forward wall approach start: speed={}'.format(speed))
        self._robot.us_turn_to(Direction.NORTH)

        last_reading = self._robot.us_distance

        # start driving
        with self._robot.driving(speed=speed) as distance:
            wait(DRIVE_WALL_POLL_INTERVAL_MS) # TODO - just in case the motors don't start immediately

            while True:
                wait(DRIVE_WALL_POLL_INTERVAL_MS)

                # check if distance limit reached
                if distance.value >= DRIVE_WALL_MAX_DISTANCE:
                    self._debug('forward wall approach reached max distance')
                    break

                # check if US reading is stable
                current_reading = self._robot.us_distance
                if math.isclose(last_reading, current_reading, abs_tol=0.1):
                    self._debug('forward wall approach stable: {:.1f}mm -> {:.1f}mm'.format(last_reading, current_reading))
                    break
                last_reading = current_reading

        # extra distance (robot is already at the wall, this just hopefully corrects the heading by ramming against the wall)
        # no need to do wait() because we are driving in one direction near-continuously
        self._robot.drive(DRIVE_WALL_EXTRA_DISTANCE, speed=speed)
        self._debug('forward wall approach complete: measured={:.1f}mm'.format(distance.value))

        # right now the odometry is all messed up
        # because the wheels have been spinning while slipping, and the robot not moving
        # so the caller should calibrate to the expected origin position


    def drive_backward_until_not_moving(self, speed: Speed = DEFAULT_STRAIGHT_SPEED):
        """Keep driving backwards (and rotating slightly) until both TS activate
        Use this to drive backward until hit wall
        NOTE: Remember to calibrate position after this!
        """
        self._debug('backward wall approach start: speed={}'.format(speed))
        with self._robot.driving(forward=False, speed=speed) as distance:
            while True:
                if self._robot.touching_rel_directions or distance.value >= DRIVE_WALL_MAX_DISTANCE:
                    # break if at least one is touching
                    break
                wait(DRIVE_WALL_POLL_INTERVAL_MS)

        self._robot.drive(DRIVE_WALL_EXTRA_DISTANCE, forward=False, speed=speed)

        if len(self._robot.touching_rel_directions) == 2:
            self._debug('backward wall approach aligned on both touch sensors')
            return

        # these are too unimportant to be constants (?)
        small_angle = 5
        small_distance = 5 # mm
        max_corrections = 5

        # no need for wait()
        corrections = 0
        while True:
            if corrections == max_corrections:
                # could be stuck, just return at this point
                break
            corrections += 1

            if len(self._robot.touching_rel_directions) == 2:
                return

            elif self._robot.is_touching_rel_direction(Direction.EAST):
                # don't use given speed here, as deltas are very small
                self._robot.turn_by(small_angle, clockwise=False)
                self._robot.drive(small_distance, forward=False)

            elif self._robot.is_touching_rel_direction(Direction.WEST):
                self._robot.turn_by(small_angle)
                self._robot.drive(small_distance, forward=False)

            self._debug('backward wall approach complete after {} corrections'.format(corrections))


    def wall_calibrate(self, current_tile: Tile, global_direction: Direction, horiz_calibrate: bool = False):
        """Calibrate the robot against the wall. Robot origin must be at tile origin.
        If horiz_calibrate specified, do horiz_calibrate on the way back to tile origin instead of a simple backwards movement.
        """
        self._debug('wall calibration start: tile={}, direction={}'.format(current_tile.map_point, global_direction))

        # calibrate only if there is a wall
        if self._map.get_wall_by_direction(current_tile, global_direction) is None:
            self._robot.us_turn_to(Direction.NORTH)
            if self._robot.is_open:
                self._debug('advance calibration skipped: no front wall')
                return

        # here, we have confirmed there is a wall directly in front of the robot
        # we can move forward and calibrate
        wait()

        # move forward until we hit the wall
        self.drive_forward_until_not_moving()

        # calibrate
        self._robot.origin = shift(current_tile.origin, global_direction, TILE_HALF_WIDTH)
        wait()

        if not horiz_calibrate:
            self._robot.drive(TILE_HALF_WIDTH, forward=False)
            self._robot.origin = current_tile.origin
            self._debug('advance calibration complete without horizontal calibration')

        else:
            self.horiz_calibrate(current_tile, global_direction)

        # calibrate
        self._robot.origin = current_tile.origin
        self._robot.heading = int(global_direction)


    def horiz_calibrate(self, current_tile: Tile, global_direction: Direction):
        """Horizontally calibrate the robot. Robot origin must be at wall."""
        self._debug('horizontal calibration start: tile={}, direction={}'.format(current_tile.map_point, global_direction))
        r'''
        Either left/right wall is required to exist
            wall
        +-+--@--+--+
        | |     |  |
  left  | |     |  |  right
  wall  | |_____|  |  wall
        +----------+
        |          |

        Diagram, assuming using left wall, and assuming robot offset to left:
        '$' represents the turning origin
                wall
        +-+--@--+-------+
        | |  $  |       |
        | |     |       |
  wall  | |     |       |
        | +-----+       |
        |               |
        +---------------+
        Move down:
        +---------------+
        | +--@--+       |
        | |  $  |       |
        | |     |       |
        | |     |       |
        | +-----+       |
        +---------------+
        Rotate (about turning origin):
        +---------------+
        | +--@--+       |
        |  \  $  \      |
        |   \     \     |
        |    \     \    |
        |     +-----+   |
        +---------------+
        Move backward:
        +-------|-------+
        |       |       |
        |       |       |
        |   +--@--+     |
        |    \  $  \    |
        |     \ |   \   |
        +------\|----\--+
        |       +-----+ |
        |       |       |
                |
                centre
        Rotate back:
        +---------------+
        |               |
        |               |
        |    +--@--+    |
        |    |  $  |    |
        |    |     |    |
        +----|-----|----+
        |    +-----+    |
        |               |
        Then move back to centre:
        ...
        '''
        left = Direction(global_direction.rotate(Direction.WEST))
        right = Direction(global_direction.rotate(Direction.EAST))

        has_left_wall = self._map.get_wall_by_direction(current_tile, left) is not None
        has_right_wall = self._map.get_wall_by_direction(current_tile, right) is not None

        if has_left_wall or has_right_wall:
            self._robot.us_turn_to(Direction.WEST if has_left_wall else Direction.EAST)

            signed_extra_distance_from_wall = self._robot.us_distance - EAST_WEST_WALL_US_DISTANCE
            offset = abs(signed_extra_distance_from_wall)

            if offset <= HORIZ_CALIBRATE_IGNORE_OFFSET:
                # normal reverse
                self._robot.drive(TILE_HALF_WIDTH, forward=False)
                self._robot.origin = current_tile.origin
                self._debug('lateral calibration ignored offset={:.1f}mm'.format(offset))

            else:
                is_too_close_to_wall = signed_extra_distance_from_wall < 0

                self._robot.drive(HORIZ_CALIBRATE_BUFFER, forward=False)
                wait()
                self._robot.turn_by(HORIZ_CALIBRATE_DEGREES, clockwise=not is_too_close_to_wall)
                distance = offset / math.sin(math.radians(HORIZ_CALIBRATE_DEGREES))
                wait()
                self._robot.drive(distance, forward=False)
                wait()
                self._robot.turn_by(HORIZ_CALIBRATE_DEGREES, clockwise=is_too_close_to_wall)
                wait()
                signed_distance_to_center = signed_distance_to(
                    self._robot.origin,
                    current_tile.origin,
                    global_direction
                )
                self._robot.drive(abs(signed_distance_to_center), forward=signed_distance_to_center > 0)

                self._robot.origin = current_tile.origin
                self._robot.heading = int(global_direction)
                self._debug('lateral calibration complete: offset={:.1f}mm'.format(offset))
        else:
            self._robot.drive(TILE_HALF_WIDTH, forward=False)
            self._robot.origin = current_tile.origin
            self._robot.heading = int(global_direction)
            self._debug('lateral calibration skipped: no side wall')
            return


    # actual movements
    # note: 
    # backtrack movements don't need nogo tile detection
    # turning movements have most potential for error, so they require wall calibration
    # advance movements also need wall calibration, but not backtrack (to save time)
    # advance movements need horiz calibration, but not backtrack (to save time)
    # advance turns need horiz calibration, but not backtrack turns (to save time) - TODO


    def advance(self, handle_nogo_tiles: bool = True, wall_calibrate: bool = True, horiz_calibrate: bool = False):
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
        self._debug('advance start: handle_nogo_tiles={}, calibrate={}, horiz_calibrate={}'.format(
            handle_nogo_tiles, wall_calibrate, horiz_calibrate
        ))
        if handle_nogo_tiles:
            self._robot.drive(ADVANCE_A_DISTANCE)
            wait()

            if self.get_smart_tile_type() == TileType.NOGO:
                self._debug('NOGO detected during straight advance')
                indicate_nogo_tile()
                self._robot.drive(ADVANCE_A_DISTANCE, forward=False)
                raise NogoTileError

            self._robot.drive(ADVANCE_B_DISTANCE)

        else:
            self._robot.drive(TILE_WIDTH)

        # we are at next tile's origin

        if wall_calibrate:
            global_direction = Direction.from_heading(self._robot.heading)
            current_tile = self._map.get_tile_by_direction(
                self._current_tile,
                global_direction,
                require_open=True
            )
            # current_tile should have been created in dfs()
            assert current_tile is not None

            self.wall_calibrate(current_tile, global_direction, horiz_calibrate)

        self._debug('advance complete')


    def backtrack(self):
        """Move backward.
        This is movement #2 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume both tiles have already been explored
        """
        # NO CALIBRATION - doesn't seem necessary atm
        self._robot.drive(TILE_WIDTH, forward=False)


    # turn movements


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
        self._debug('turn step 1: inverse={}'.format(inverse))
        if not inverse:
            self._robot.drive(STEP_1_DISTANCE, forward=False)
        else:
            self._robot.drive(STEP_1_DISTANCE)


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
        self._debug('turn step 2: inverse={}'.format(inverse))
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
        self._debug('turn step 3: inverse={}'.format(inverse))
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
        self._debug('turn step 4: inverse={}'.format(inverse))
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
        self._debug('turn step 5 no calibration: inverse={}, raise_if_nogo={}'.format(inverse, raise_if_nogo))
        if not inverse:
            self._robot.drive(STEP_5_A_DISTANCE)

            if raise_if_nogo and self.get_smart_tile_type() == TileType.NOGO:
                self._debug('NOGO detected during diagonal step 5')
                wait()
                self._robot.drive(STEP_5_A_DISTANCE, forward=False)
                raise NogoTileError

            self._robot.drive(STEP_5_B_DISTANCE)

        else:
            distance = STEP_5_A_DISTANCE + STEP_5_B_DISTANCE
            self._robot.drive(distance, forward=False)


    def _turn_step_5(self, inverse=False, raise_if_nogo=True, wall_calibrate=True):
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
        self._debug('turn step 5: inverse={}, raise_if_nogo={}, wall_calibrate={}'.format(
            inverse, raise_if_nogo, wall_calibrate
        ))
        if not wall_calibrate:
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
                self._debug('step 5 calibration skipped: no rear wall')
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
                self._robot.drive(STEP_5_DISTANCE + STEP_5_A_DISTANCE - TILE_HALF_WIDTH)

                if raise_if_nogo and self.get_smart_tile_type() == TileType.NOGO:
                    self._debug('NOGO detected during calibrated step 5')
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
                self._robot.drive(STEP_5_DISTANCE - (ROBOT_HEIGHT - TILE_HALF_WIDTH))


    def advance_right(self, handle_nogo_tiles: bool = True, wall_calibrate: bool = True):
        """Move forward and to the right.
        This is movement #4 in the doc.
        Raise black tile error and return to original position if black tile detected.
        """
        self._debug('advance right start: handle_nogo_tiles={}, calibrate={}'.format(handle_nogo_tiles, wall_calibrate))
        self._turn_step_1()
        wait()
        self._turn_step_2()
        wait()
        self._turn_step_3()
        wait()
        # TODO: technically, if we rotate and find a black tile, we're not aloud to rotate back out of it.
        self._turn_step_4()
        wait()

        if handle_nogo_tiles and self.get_smart_tile_type() == TileType.NOGO: # no need for smart()
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
            self._turn_step_5(raise_if_nogo=handle_nogo_tiles, wall_calibrate=wall_calibrate)
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


    def advance_left(self, handle_nogo_tiles: bool = True, wall_calibrate: bool = True):
        """Move forward and to the left.
        This is movement #3 in the doc.
        Raise black tile error and return to original position if black tile detected.
        """
        self._debug('advance left start: handle_nogo_tiles={}, calibrate={}'.format(handle_nogo_tiles, wall_calibrate))
        # TODO: handle calibration
        self._turn_step_1()
        wait()
        self._turn_step_2(inverse=True)
        wait()
        self._turn_step_3()
        wait()
        self._turn_step_4(inverse=True)
        wait()

        if handle_nogo_tiles and self.get_smart_tile_type() == TileType.NOGO:
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
            self._turn_step_5(raise_if_nogo=handle_nogo_tiles, wall_calibrate=wall_calibrate)
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
            raise


    def backtrack_right(self, wall_calibrate: bool = True):
        """Move backward and to the right.
        This is movement #6 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume all three tiles have already been explored
        """
        self._debug('backtrack right start: calibrate={}'.format(wall_calibrate))
        self._turn_step_5(inverse=True, raise_if_nogo=False, wall_calibrate=wall_calibrate)
        wait()
        self._turn_step_4(inverse=True)
        wait()
        self._turn_step_3(inverse=True)
        wait()
        self._turn_step_2(inverse=True)
        wait()
        self._turn_step_1(inverse=True)


    def backtrack_left(self, wall_calibrate: bool = True):
        """Move backward and to the left.
        This is movement #5 in the doc.
        NOTE: there is no need to worry about black tiles,
        because we assume all three tiles have already been explored
        """
        self._debug('backtrack left start: calibrate={}'.format(wall_calibrate))
        self._turn_step_5(inverse=True, raise_if_nogo=False, wall_calibrate=wall_calibrate)
        wait()
        self._turn_step_4()
        wait()
        self._turn_step_3(inverse=True)
        wait()
        self._turn_step_2()
        wait()
        self._turn_step_1(inverse=True)




