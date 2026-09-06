"""Belief-over-box-location representation.

A SearchRegion is a uniform belief over a rectangle on the table plane: "the
box is somewhere in here, each spot equally likely." That's what "true
position plus a margin of uncertainty" (see belief_region_node.py) actually
means as a prior. Each robot picks ONE random point in its region and
physically tries to pick there - a single guess, not an iterative search -
so a wide region (no help) makes a lucky guess unlikely, and a narrow region
(help) makes one all but certain, without needing the guess to be "smart"
about where within the region to try.
"""
import random
from dataclasses import dataclass
from typing import Tuple

from geometry_msgs.msg import Point32, Polygon, PolygonStamped


@dataclass
class SearchRegion:
    center_x: float
    center_y: float
    half_extent_x: float
    half_extent_y: float
    z: float  # world-frame height of the table surface / box center

    def sample_point(self) -> Tuple[float, float]:
        """One uniformly-random guess within this belief rectangle."""
        x = self.center_x + random.uniform(-self.half_extent_x, self.half_extent_x)
        y = self.center_y + random.uniform(-self.half_extent_y, self.half_extent_y)
        return x, y

    def to_polygon_stamped(self, frame_id: str = "world") -> PolygonStamped:
        msg = PolygonStamped()
        msg.header.frame_id = frame_id
        corners = [
            (self.center_x - self.half_extent_x, self.center_y - self.half_extent_y),
            (self.center_x + self.half_extent_x, self.center_y - self.half_extent_y),
            (self.center_x + self.half_extent_x, self.center_y + self.half_extent_y),
            (self.center_x - self.half_extent_x, self.center_y + self.half_extent_y),
        ]
        msg.polygon = Polygon(points=[Point32(x=float(x), y=float(y), z=float(self.z)) for x, y in corners])
        return msg

    @classmethod
    def from_polygon_stamped(cls, msg: PolygonStamped) -> "SearchRegion":
        xs = [p.x for p in msg.polygon.points]
        ys = [p.y for p in msg.polygon.points]
        zs = [p.z for p in msg.polygon.points]
        return cls(
            center_x=(min(xs) + max(xs)) / 2,
            center_y=(min(ys) + max(ys)) / 2,
            half_extent_x=(max(xs) - min(xs)) / 2,
            half_extent_y=(max(ys) - min(ys)) / 2,
            z=sum(zs) / len(zs),
        )


def region_from_true_pose(x: float, y: float, z: float, margin: float) -> SearchRegion:
    """The oracle's belief construction: true position +/- a square margin."""
    return SearchRegion(center_x=x, center_y=y, half_extent_x=margin, half_extent_y=margin, z=z)
