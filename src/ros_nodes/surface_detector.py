#!/usr/bin/env python3
"""Publish terrain surface type under the robot pose."""

from __future__ import annotations

import json
from dataclasses import dataclass
from numbers import Integral
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import rospy
from nav_msgs.msg import Odometry
from shapely.geometry import Point, shape
from shapely.strtree import STRtree
from std_msgs.msg import String


UNKNOWN_SURFACE = "unknown"


class SurfaceMap:
    map_type = "base"

    def surface_at(self, x: float, y: float) -> str:
        raise NotImplementedError


class SingleSurfaceMap(SurfaceMap):
    map_type = "single_surface"

    def __init__(self, surface_name: str) -> None:
        if not surface_name:
            raise ValueError("single surface map requires a non-empty surface name")
        self._surface_name = str(surface_name)

    @classmethod
    def from_json(cls, data: dict) -> "SingleSurfaceMap":
        surface_name = data.get("surface_name")
        if surface_name is None:
            surface_names = data.get("surface_names", [])
            surface_name = surface_names[0] if surface_names else None
        return cls(surface_name)

    def surface_at(self, x: float, y: float) -> str:
        return self._surface_name


@dataclass(frozen=True)
class Bounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def contains(self, x: float, y: float, eps: float = 1e-9) -> bool:
        return (
            self.min_x - eps <= x <= self.max_x + eps
            and self.min_y - eps <= y <= self.max_y + eps
        )


class SquareSurfaceMap(SurfaceMap):
    map_type = "squares"

    def __init__(
        self,
        grid: Sequence[Sequence[str]],
        tile_size: float,
        origin: Tuple[float, float],
        bounds: Bounds,
    ) -> None:
        if tile_size <= 0.0:
            raise ValueError("tile_size must be positive")
        if not grid or not grid[0]:
            raise ValueError("square surface map cannot be empty")

        self._grid = [list(row) for row in grid]
        self._tile_size = tile_size
        self._origin_x, self._origin_y = origin
        self._bounds = bounds
        self._rows = len(self._grid)
        self._cols = len(self._grid[0])

        for row in self._grid:
            if len(row) != self._cols:
                raise ValueError("all square map rows must have the same length")

    @classmethod
    def from_json(cls, data: dict) -> "SquareSurfaceMap":
        grid = data["map"]
        tile_size = float(data["tile_size"])
        rows = len(grid)
        cols = len(grid[0]) if rows else 0
        map_width = float(data.get("map_width", cols * tile_size))
        map_length = float(data.get("map_length", rows * tile_size))

        origin_data = data.get("origin", {})
        origin = (
            float(origin_data.get("x", -map_width / 2.0)),
            float(origin_data.get("y", -map_length / 2.0)),
        )
        bounds = parse_bounds(
            data,
            default=Bounds(
                origin[0],
                origin[0] + map_width,
                origin[1],
                origin[1] + map_length,
            ),
        )
        return cls(grid, tile_size, origin, bounds)

    def surface_at(self, x: float, y: float) -> str:
        if not self._bounds.contains(x, y):
            return UNKNOWN_SURFACE

        col = int((x - self._origin_x) // self._tile_size)
        row = int((y - self._origin_y) // self._tile_size)

        if col == self._cols and abs(x - self._bounds.max_x) <= 1e-9:
            col = self._cols - 1
        if row == self._rows and abs(y - self._bounds.max_y) <= 1e-9:
            row = self._rows - 1

        if 0 <= row < self._rows and 0 <= col < self._cols:
            return str(self._grid[row][col])
        return UNKNOWN_SURFACE


class VoronoiSurfaceMap(SurfaceMap):
    map_type = "voronoi"

    def __init__(
        self,
        geometries: Sequence[dict],
        surface_names: Sequence[str],
        bounds: Bounds | None,
    ) -> None:
        if len(geometries) != len(surface_names):
            raise ValueError("voronoi map and surface_map must have equal lengths")
        if not geometries:
            raise ValueError("voronoi surface map cannot be empty")

        self._polygons = [shape(item) for item in geometries]
        self._surface_names = [str(name) for name in surface_names]
        self._bounds = bounds
        self._tree = STRtree(self._polygons)
        self._geometry_index: Dict[int, int] = {
            id(geometry): index for index, geometry in enumerate(self._polygons)
        }

    @classmethod
    def from_json(cls, data: dict) -> "VoronoiSurfaceMap":
        return cls(
            data["map"],
            data["surface_map"],
            parse_bounds(data, default=None),
        )

    def surface_at(self, x: float, y: float) -> str:
        if self._bounds is not None and not self._bounds.contains(x, y):
            return UNKNOWN_SURFACE

        point = Point(x, y)
        for candidate in self._tree.query(point):
            index = self._candidate_index(candidate)
            polygon = self._polygons[index]
            if polygon.covers(point):
                return self._surface_names[index]
        return UNKNOWN_SURFACE

    def _candidate_index(self, candidate) -> int:
        if isinstance(candidate, Integral):
            return int(candidate)

        index = self._geometry_index.get(id(candidate))
        if index is not None:
            return index

        for fallback_index, polygon in enumerate(self._polygons):
            if polygon.equals(candidate):
                return fallback_index
        raise RuntimeError("STRtree returned geometry that is not in the map")


class SurfaceDetector:
    def __init__(self) -> None:
        self._world_name = str(rospy.get_param("/world_name", "world"))
        self._world_path = str(rospy.get_param("/world_path", ""))
        self._map_path = resolve_surface_map_path(
            explicit_path=str(rospy.get_param("~surface_map_path", "")),
            world_path=self._world_path,
            world_name=self._world_name,
        )
        self._surface_map = load_surface_map(self._map_path)
        self._surface_topic = rospy.get_param("~surface_topic", "/terrain/surface_type")
        self._odom_topic = rospy.get_param("~odom_topic", "/ground_truth/odom")

        self._publisher = rospy.Publisher(self._surface_topic, String, queue_size=1)
        rospy.Subscriber(self._odom_topic, Odometry, self._odom_callback)

        rospy.loginfo(
            "Surface detector ready: map=%s type=%s odom=%s topic=%s",
            self._map_path,
            self._surface_map.map_type,
            self._odom_topic,
            self._surface_topic,
        )

    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        surface = self._surface_map.surface_at(x, y)

        self._publisher.publish(surface)

        rospy.loginfo_throttle(
            1.0,
            "pos=(%.2f, %.2f) surface=%s",
            x,
            y,
            surface,
        )


def load_surface_map(map_path: Path) -> SurfaceMap:
    with map_path.open("r") as map_file:
        data = json.load(map_file)

    map_type = data.get("map_type")
    if map_type == SingleSurfaceMap.map_type:
        return SingleSurfaceMap.from_json(data)
    if map_type == SquareSurfaceMap.map_type:
        return SquareSurfaceMap.from_json(data)
    if map_type == VoronoiSurfaceMap.map_type:
        return VoronoiSurfaceMap.from_json(data)
    raise ValueError(f"Unsupported surface map type: {map_type}")


def resolve_surface_map_path(
    explicit_path: str,
    world_path: str,
    world_name: str,
) -> Path:
    if explicit_path:
        return checked_path(Path(explicit_path).expanduser())

    if world_path:
        path = Path(world_path).expanduser()
        if path.suffix == ".world":
            return checked_path(path.with_name(f"{path.stem}_surface_map.json"))

    world_as_path = Path(world_name).expanduser()
    if world_as_path.suffix == ".world":
        return checked_path(
            world_as_path.with_name(f"{world_as_path.stem}_surface_map.json")
        )

    package_dir = Path(__file__).resolve().parents[1]
    candidate = (
        package_dir
        / "worlds_library"
        / world_name
        / f"{world_name}_surface_map.json"
    )
    return checked_path(candidate)


def checked_path(path: Path) -> Path:
    resolved = path.resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"surface map file does not exist: {resolved}")
    return resolved


def parse_bounds(data: dict, default):
    bounds = data.get("bounds")
    if not bounds:
        return default
    return Bounds(
        min_x=float(bounds["min_x"]),
        max_x=float(bounds["max_x"]),
        min_y=float(bounds["min_y"]),
        max_y=float(bounds["max_y"]),
    )


def main() -> None:
    rospy.init_node("surface_detector")
    SurfaceDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
