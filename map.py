#!/usr/bin/env python3

import attr
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple, Union

from shapely.geometry import Point, LineString

from constants import *
from utils import *

__all__ = ['Tile', 'Wall', 'Map']


@attr.s
class Tile:
    """This represents a maze tile. Tiles have the following properties:
    map_point: a coordinate representing the location of the tile in the map coordinate system.
    tile_type: an optional TileType representing what tile it is, if known.
    visited: a bool flag storing whether the robot has physically been to this tile.

    Here is an example of the multiple tile coordinate systems at play:
                                |
                                |
                                |
                            |‾‾‾|‾‾‾|
                            | (0,1) |
                            |   |   |
                    |‾‾‾‾‾‾‾|‾‾‾|‾‾‾|‾‾‾‾‾‾‾|
            --------| (-1,0)| (0,0) |   @=======
                    |_______|___|___|_______|
                    |       |   |   |
                    |(-1,-1)| (0,-1)|   %
                    |_______|___|___|
                                |
                                |
                                |
    The robot (represented by a line, origin is at '@')
    - is on tile coordinate (1,0)
    - is on global coordinate (<TILE WIDTH MM>, 0)
    - is on relative coordinate (0, 0) (because the relative origin moves with the robot)
    - has a heading of 270deg (West)

    The surrounding point '%'
    - isn't on a tile
    - is on global coordinate (<TILE WIDTH MM>, - <TILE WIDTH MM>)
    - is on relative coordinate (- <TILE WIDTH MM>, 0) (relative to the origin and heading)
    """

    map_point = attr.ib(on_setattr=attr.setters.frozen) # type: Point

    tile_type = attr.ib(default=None) # type: Optional[TileType]

    visited = attr.ib(default=False) # type: bool

    if TYPE_CHECKING:
        def __init__(self, map_point: Point, tile_type: Optional[TileType] = None, visited: bool = False): ...


    @property
    def origin(self) -> Point:
        """Return the global coordinates of the tile center. E.g. (0,1) -> (0, <TILE WIDTH MM>)"""
        return Point(
            self.map_point.x * TILE_WIDTH,
            self.map_point.y * TILE_WIDTH
        )


    def is_adjacent_to(self, tile: 'Tile') -> bool:
        """Check whether this tile is adjacent to another, i.e. one is directly n/s/e/w of the other"""
        is_shifted_x = abs(self.map_point.x - tile.map_point.x) == 1
        is_shifted_y = abs(self.map_point.y - tile.map_point.y) == 1
        return is_shifted_x != is_shifted_y # one is true, one is false


@attr.s(frozen=True)
class Wall:
    """This represents a maze wall.
    +----$-----+
    |          |
    |    @     #
    |          |
    +----------+
    In above example, if '@' is Tile(0, 0)
    then '$' is Wall(0, 0.5) and '#' is Wall(0.5, 0)
    (in map coordinates).
    """

    map_point = attr.ib() # type: Point

    if TYPE_CHECKING:
        def __init__(self, map_point: Point): ...


    @property
    def origin(self) -> Point:
        """Return the global coordinates of the wall midpoint. E.g. (0,0.5) -> (0, <TILE WIDTH MM>/2)"""
        return Point(
            self.map_point.x * TILE_WIDTH,
            self.map_point.y * TILE_WIDTH
        )


    def is_between(self, a: Tile, b: Tile) -> bool:
        """Does this wall separate the two tiles?"""
        assert a.is_adjacent_to(b)
        midpoint = LineString([a.map_point, b.map_point]).centroid
        return self.map_point == midpoint


    @classmethod
    def from_tiles(cls, a: Tile, b: Tile) -> 'Wall':
        """Create a wall between two tiles"""
        assert a.is_adjacent_to(b)
        midpoint = LineString([a.map_point, b.map_point]).centroid
        return cls(midpoint)


    @classmethod
    def from_tile(cls, tile: Tile, direction: Direction) -> 'Wall':
        """Create a wall on the <direction> side of <tile>"""
        midpoint = shift(tile.map_point, direction, 0.5)
        return cls(midpoint)


class Map:
    """This is the robot's 'memory'.
    The robot can quickly access which tile it's on, and which tiles are open.
    When it discovers things (e.g. new tiles, victims), it should update this map.
    """

    def __init__(self):
        # Internal map
        self._tiles = [] # type: List[Tile]
        self._walls = [] # type: List[Wall]


    @property
    def is_empty(self) -> bool:
        return not bool(self._tiles)


    # tiles


    def add_tile(self, tile: Tile):
        """Add a tile to the map. Silent if already added"""
        if self.get_tile(tile.map_point) is None: # prefer over `tile in self._tiles`
            self._tiles.append(tile)


    def add_tiles(self, *tiles: Tile):
        """Add multiple tiles to the map. Silent if already added"""
        for tile in tiles:
            self.add_tile(tile)


    def get_tile(self, map_point: Point) -> Optional[Tile]:
        """Lookup a tile from its tile coordinate"""
        return next((tile for tile in self._tiles if tile.map_point == map_point), None)


    def get_tile_by_direction(self, tile: Tile, direction: Direction, require_open: bool) -> Optional[Tile]:
        """Lookup a tile from its direction relative to another tile.
        In other words, moving to the <direction> of <tile> gives us the result (or None).
        NOTE: there could be a wall between the two tiles. To only consider tiles that are open, use require_open=True
        """
        new_tile = self.get_tile(shift(tile.map_point, direction))

        if new_tile is None:
            return None

        if require_open and not self.is_open(tile, new_tile):
            return None

        return new_tile


    def create_tile_by_direction(self, tile: Tile, direction: Direction) -> Tile:
        """Create a tile to the <direction> of <tile> and add it to the map. Returns tile if it exists"""
        new_tile = self.get_tile(shift(tile.map_point, direction))
        if new_tile is None:
            new_tile = Tile(shift(tile.map_point, direction))
            self.add_tile(new_tile)
        return new_tile


    # walls


    def add_wall(self, wall: Wall):
        """Add a wall to the map. Silent if already added"""
        if self.get_wall(wall.map_point) is None:
            self._walls.append(wall)


    def get_wall(self, map_point: Point) -> Optional[Wall]:
        return next((wall for wall in self._walls if wall.map_point == map_point), None)


    def get_wall_by_direction(self, tile: Tile, direction: Direction) -> Optional[Wall]:
        map_point = shift(tile.map_point, direction, 0.5)
        return next((wall for wall in self._walls if wall.map_point == map_point), None)


    def create_wall_by_direction(self, tile: Tile, direction: Direction) -> 'Wall':
        """Create a wall to the <direction> of <tile>. Return wall if it exists"""
        wall = Wall.from_tile(tile, direction)
        self.add_wall(wall)
        return wall


    def create_wall_between(self, a: Tile, b: Tile) -> 'Wall':
        """Create a wall betweeen two tiles. Return wall if it exists"""
        wall = Wall.from_tiles(a, b)
        self.add_wall(wall)
        return wall


    # utils


    def is_open(self, a: Tile, b: Tile) -> bool:
        """Check whether two adjacent tiles are open (have no wall separating them)
        We check whether an wall exists
        """
        assert a.is_adjacent_to(b)
        wall = next((wall for wall in self._walls if wall.is_between(a, b)), None)
        if wall is None:
            return True
        return False


    def mark_open(self, a: Tile, b: Tile):
        """Mark two tiles as open, by removing a wall between them if any. Silent if already open"""
        assert a.is_adjacent_to(b)
        self._walls = [wall for wall in self._walls if not wall.is_between(a, b)]


    def mark_nogo(self, tile: Tile):
        """Call this when the robot detects that a tile is black. Silent if already walled.
        We change the tile's `tile_type` to black, and add walls all around it
        """
        tile.tile_type = TileType.NOGO
        for direction in Direction:
            wall = Wall.from_tile(tile, direction)
            self.add_wall(wall)


    @property
    def statistics(self) -> Dict[TileType, int]:
        """Get the statistics of the run. Robot should announce this at the end of the solve.
        NOTE: do not access this property during the solve, because some tiles might have incomplete information
        """
        stats = dict.fromkeys(TileType, 0) # fromkeys - very neat

        for tile in self._tiles:
            # every tile *should* have a tiletype by now
            if tile.tile_type is None: continue

            stats[tile.tile_type] += 1

        return stats
    
