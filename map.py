#!/usr/bin/env python3

import attr
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple, Union

from shapely.geometry import Point

from constants import *
from utils import *

__all__ = [
    'Tile',
    'Edge',
    'Map'
]



@attr.s
class Tile:
    """This represents a maze tile. Tiles have the following properties:
    tile_point: a coordinate representing the location of the tile in the tile coordinate system.
    tile_type: an optional TileType representing what tile it is, if known.
    victim_type: an optional VictimType representing what victim is in the tile, if known.
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

    tile_point = attr.ib(on_setattr=attr.setters.frozen) # type: Point

    tile_type = attr.ib(default=None) # type: Optional[TileType]

    victim_type = attr.ib(default=None) # type: Optional[VictimType]

    visited = attr.ib(default=False) # type: bool

    if TYPE_CHECKING:
        def __init__(self, tile_point: Point, tile_type: TileType, visited: bool = False): ...


    @property
    def origin(self) -> Point:
        """Return the global coordinates of the tile center. E.g. (0,1) -> (0, <TILE WIDTH MM>)"""
        return Point(
            self.tile_point.x * TILE_WIDTH,
            self.tile_point.y * TILE_WIDTH
        )


    def is_adjacent_to(self, tile: 'Tile') -> bool:
        """Check whether this tile is adjacent to another, i.e. one is directly n/s/e/w of the other"""
        is_shifted_x = abs(self.tile_point.x - tile.tile_point.x) == 1
        is_shifted_y = abs(self.tile_point.y - tile.tile_point.y) == 1
        return is_shifted_x != is_shifted_y # one is true, one is false



@attr.s(frozen=True, auto_attribs=True)
class Edge:
    """This represents an undirected edge between two adjacent tiles.
    a: a Tile
    b: a Tile

    Here is an example of a tile-edge graph based on a maze:
            |‾‾‾‾‾‾‾|
            | (0,1) |
            |       |
    |‾‾‾‾‾‾‾        ‾‾‾‾‾‾‾‾|
    | (-1,0)  (0,0)   (1,0) |
    |_______        ________|
    |               |
    |(-1,-1)  (0,-1)|
    |_______________|
    
    becomes:

                (0,1)
                  |
                  |
     (-1,0)-----(0,0)-----(1,0)
                  |
                  |
    (-1,-1)-----(0,-1)

    Each point is a Tile, each line is an Edge.
    Each edge represents an open direction.
    """

    a = attr.ib() # type: Tile
    b = attr.ib() # type: Tile

    if TYPE_CHECKING:
        def __init__(self, a: Tile, b: Tile) -> None: ...

    @b.validator # type: ignore
    def _validate_tiles(self, _, b: Tile):
        a = self.a
        assert a != b
        assert a.tile_point != b.tile_point
        assert (
            (abs(self.a.tile_point.x - self.b.tile_point.x) == 1)
            != (abs(self.a.tile_point.y - self.b.tile_point.y) == 1)
        ) # tiles must be adjacent

    # this makes `Edge(a, b) == Edge(b, a) work`
    def __eq__(self, other):
        if not isinstance(other, type(self)):
            return NotImplemented
        return self.has(*other.tiles)


    @property
    def tiles(self) -> Tuple[Tile, Tile]:
        """Tuple of the tiles"""
        return self.a, self.b


    def has(self, *tiles: Tile) -> bool:
        """Check whether the edge has these tiles"""
        return all(tile in self.tiles for tile in tiles)



class Map:
    """This is the robot's 'memory'.
    It can quickly access which tile it's on, and which tiles are open.
    When it discovers things (e.g. new tiles, victims), it should update this map.
    """

    def __init__(self):
        # Internal storage of tiles and edges
        self._tiles = [] # type: List[Tile]
        self._edges = [] # type: List[Edge]


    # tile operations


    def add_tile(self, tile: Tile):
        """Add a tile to the map"""
        assert self.get_tile(tile.tile_point) is None # prefer over `tile in self._tiles` because tile_type may change
        self._tiles.append(tile)


    def get_tile(self, tile_point: Point) -> Optional[Tile]:
        """Lookup a tile from its tile coordinate"""
        return next((tile for tile in self._tiles if tile.tile_point == tile_point), None)


    def get_tile_by_direction(self, tile: Tile, direction: Direction) -> Optional[Tile]:
        """Lookup a tile from its direction relative to another tile.
        In other words, moving to the <direction> of <tile> gives us the result (or None).
        NOTE: there could be a wall between the two tiles.
        If using this function to find tiles to move to, always check whether that tile's blocked.
        """
        return self.get_tile(shift(tile.tile_point, direction))


    # walls / open paths


    def is_open(self, a: Tile, b: Tile) -> bool:
        """Check whether two adjacent tiles are open (have no wall separating them)
        We check whether an edge exists
        """
        assert a.is_adjacent_to(b)
        edge = next((edge for edge in self._edges if edge.has(a, b)), None)
        if edge is None:
            return False
        return True


    def is_open_direction(self, tile: Tile, direction: Direction) -> bool:
        """Check whether a direction is clear, i.e. can the robot move in this direction without bumping into a wall
        We check whether an edge exists between the two tiles.
        NOTE: if <direction> leads off the map, i.e. there is no tile in that direction, we return False
        """
        target = self.get_tile_by_direction(tile, direction)
        if target is None:
            return False
        
        if not self.is_open(tile, target):
            return False

        return True


    def mark_open(self, a: Tile, b: Tile):
        """Call this when the robot discovers an open path between two tiles.
        We add an edge between the tiles
        """
        edge = Edge(a, b)
        if edge not in self._edges:
            self._edges.append(edge)


    def mark_closed(self, a: Tile, b: Tile):
        """Call this when the robot discovers a wall between two tiles.
        We remove the edge between them, if one exists.
        """
        self._edges = [edge for edge in self._edges if not edge.has(a, b)]


    def mark_black(self, tile: Tile):
        """Call this when the robot detects that a tile is black.
        We change the tile's `tile_type` to black, and remove all edges (open paths) containing this tile
        """
        tile.tile_type = TileType.BLACK
        self._edges = [edge for edge in self._edges if not edge.has(tile)]


    # statistics, TODO


    @property
    def statistics(self) -> Dict[Union[TileType, VictimType], int]:
        """Get the statistics of the run. Robot should announce this at the end of the solve.
        NOTE: do not access this property during the solve, because some tiles might have incomplete information
        """
        normal_tiles = sum(1 for tile in self._tiles if tile.tile_type == TileType.NORMAL)
        black_tiles = sum(1 for tile in self._tiles if tile.tile_type == TileType.BLACK)

        harmed_victims = sum(1 for tile in self._tiles if tile.victim_type == VictimType.HARMED)
        unharmed_victims = sum(1 for tile in self._tiles if tile.victim_type == VictimType.UNHARMED)

        return {
            TileType.NORMAL: normal_tiles,
            TileType.BLACK: black_tiles,
            VictimType.HARMED: harmed_victims,
            VictimType.UNHARMED: unharmed_victims
        }

