#!/usr/bin/env python3

from shapely.geometry import Point

from constants import *
from utils import *

from robot import Robot
from map import Tile, Map
from solver import Solver

# do not remove:
import button
import indicate

__all__ = ['RobotTester', 'SolverTester']



class RobotTester:
    def __init__(self, robot: Robot):
        self._robot = robot

        self.calibrate_to_zero()


    def calibrate_to_north(self):
        self._robot.heading = 0


    def calibrate_to_global_origin(self):
        self._robot.origin = Point(0, 0)


    def calibrate_to_zero(self):
        """REMEMBER TO CALL THIS AFTER EVERY TEST
        IF YOU MOVE THE ROBOT BACK WITH YOUR HANDS
        Don't make my fucking mistake and run multiple tests consectively
        and wonder why things are suddenly breaking.
        """
        self._robot.heading = 0
        self._robot.origin = Point(0, 0)


    # basic tests - can the robot do these things?


    def test_relative_turning(self):
        """Expect a 90deg clockwise turn on the spot."""
        self._robot.turn_by(90)
        self._robot.log()


    def test_absolute_turning(self):
        """Expect a turn ccw, then a turn exactly North"""
        self._robot.turn_by(45, clockwise=False)
        wait()
        self._robot.turn_to(Direction.NORTH)
        self._robot.log()


    def test_multiple_absolute_turning(self):
        """Expect multiple turns, then a turn exactly West"""
        print(self._robot.heading)
        self._robot.turn_by(45, clockwise=False)
        print(self._robot.heading)
        wait()
        self._robot.turn_by(80)
        print(self._robot.heading)
        wait()
        self._robot.turn_by(30, clockwise=False)
        print(self._robot.heading)
        wait()
        self._robot.turn_to(Direction.WEST)
        print(self._robot.heading)
        self._robot.log()


    def test_driving_straight(self):
        """Expect a 10cm drive backward then forward"""
        self._robot.drive(100, forward=False)
        self._robot.drive(100, forward=True)
        self._robot.log()


    def test_us_distance(self):
        print(self._robot.us_distance)


    def test_us_turning(self):
        """Expect an absolute turn to East"""
        self._robot.us_turn_to(Direction.EAST)
        self._robot.log()


    def test_us_calibration(self):
        self._robot.us_calibrate_to_north()
        self._robot.log()


    def test_us_lookaround(self):
        print(self._robot.lookaround())


    def test_color(self):
        print(self._robot.color)


    def test_type(self):
        print(self._robot.tile_type)


    # calibration tests
    #TODO


class SolverTester:
    def __init__(self, solver: Solver):
        self._solver = solver
        self._robot = solver._robot
        self._map = solver._map

        self.calibrate_to_zero()

    def calibrate_to_direction(self, direction: Direction):
        self._robot.heading = direction


    def calibrate_to_north(self):
        self._robot.heading = 0


    def calibrate_to_map_point(self, map_point: Point):
        tile = self._map.get_tile(map_point)
        assert tile is not None
        self._solver._current_tile = tile
        self._robot.origin = tile.origin


    def calibrate_to_global_origin(self):
        self.calibrate_to_map_point(Point(0, 0))
        self._robot.origin = Point(0, 0)


    def calibrate_to_zero(self):
        """REMEMBER TO CALL THIS AFTER EVERY TEST
        IF YOU MOVE THE ROBOT BACK WITH YOUR HANDS
        """
        self.calibrate_to_north()
        self.calibrate_to_global_origin()


    # basic tests


    def test_advance(self, test_nogo_tiles=False, test_calibration=False, test_us_calibration=False):
        self._solver.advance(test_nogo_tiles, test_calibration, test_us_calibration)
        self._robot.log()


    def test_backtrack(self):
        self._solver.backtrack()
        self._robot.log()


    def test_advance_left(self, test_nogo_tiles=False, test_calibration=False):
        self._solver.advance_left(test_nogo_tiles, test_calibration)
        self._robot.log()


    def test_advance_right(self, test_nogo_tiles=False, test_calibration=False):
        self._solver.advance_right(test_nogo_tiles, test_calibration)
        self._robot.log()


    def test_backtrack_left(self, test_calibration=False):
        self._solver.backtrack_left(test_calibration)
        self._robot.log()


    def test_backtrack_right(self, test_calibration=False):
        self._solver.backtrack_right(test_calibration)
        self._robot.log()


    def test_drive_forward_wall(self):
        self._solver.drive_forward_until_not_moving()
        self._robot.log()


    def test_drive_backward_wall(self):
        self._solver.drive_backward_until_not_moving()
        self._robot.log()



if __name__ == '__main__':
    input('Press enter to begin tests')

    robot = Robot()
    robot.init()

    try:
        robtest = RobotTester(robot)

        map = Map()
        '''
             +----+
        wall |    |
        +-|--+----+----+
        | |==|    |    |
        +----+----+----+
             |    | ← (0,0) current tile
             +----+
        '''
        current_tile = Tile(Point(0, 0))
        map.add_tile(current_tile)
        north_tile = map.create_tile_by_direction(current_tile, Direction.NORTH)
        north_tile_west_wall = map.create_wall_by_direction(north_tile, Direction.WEST)
        north_east_tile = map.create_tile_by_direction(north_tile, Direction.EAST)
        north_west_tile = map.create_tile_by_direction(north_tile, Direction.WEST)
        north_north_tile = map.create_tile_by_direction(north_tile, Direction.NORTH)

        solver = Solver(robot, map)

        solvetest = SolverTester(solver)

        # globals:
        # robot, robtest, map, solver, solvetest, button, indicate
        # (and constants.*)

        while True:
            string = input('Enter command (or nothing to exit): ')
            if not string: break

            try:
                exec(string, globals())
            except Exception as e:
                print(e)

    finally:
        robot.shutdown()
