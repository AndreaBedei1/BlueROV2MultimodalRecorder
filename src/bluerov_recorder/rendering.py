"""Fast cached Surveyor fan rasterization for the GUI preview only."""

from __future__ import annotations

import math
from functools import lru_cache
from typing import Any, Dict, Tuple

import numpy as np
from PIL import Image, ImageDraw

try:
    import cv2
except ImportError:  # pragma: no cover - Pillow fallback remains in app.py.
    cv2 = None


BACKGROUND = np.asarray((6, 16, 29), dtype=np.uint8)


@lru_cache(maxsize=32)
def _fan_maps(width: int, height: int, beams: int, bins: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int, int, int]:
    center_x = int(width) // 2
    origin_y = int(height) - 35
    max_radius = min(center_x - 25, int(height) - 65)
    yy, xx = np.indices((int(height), int(width)), dtype=np.float32)
    dx = xx - np.float32(center_x)
    dy = np.float32(origin_y) - yy
    radius = np.sqrt(dx * dx + dy * dy)
    angle_deg = np.rad2deg(np.arctan2(dx, dy))
    valid = (
        (dy >= 0.0) & (radius <= float(max_radius))
        & (angle_deg >= -40.0) & (angle_deg <= 40.0)
    )
    beam_map = (angle_deg + 40.0) * np.float32(max(1, beams - 1) / 80.0)
    range_map = radius * np.float32(max(1, bins - 1) / max(1, max_radius))
    return range_map, beam_map, valid, center_x, origin_y, max_radius


@lru_cache(maxsize=1)
def _colour_lut() -> np.ndarray:
    values = np.arange(256, dtype=np.float32) / 255.0
    red = np.clip((values - 0.36) * 2.1, 0.0, 1.0)
    green = np.clip((values - 0.08) * 1.35, 0.0, 1.0)
    blue = np.clip(0.22 + values * 1.1, 0.0, 1.0)
    return np.stack((red, green, blue), axis=1).astype(np.float32).__mul__(255.0).astype(np.uint8)


def render_surveyor_fan(
    record: Dict[str, Any],
    width: int,
    height: int,
    brightness: float = 1.0,
    contrast: float = 1.0,
    show_atof: bool = True,
) -> Image.Image:
    """Render a matrix with one cached OpenCV remap instead of polygons."""
    matrix = np.asarray(record.get("matrix") or [], dtype=np.float32)
    if matrix.ndim != 2 or not matrix.size or cv2 is None:
        raise ValueError("vectorized fan rendering unavailable")
    finite = matrix[np.isfinite(matrix)]
    if not finite.size:
        raise ValueError("fan matrix contains no finite values")
    low, high = np.percentile(finite, (5.0, 99.0))
    if high <= low:
        high = low + 1.0
    normalized = (matrix - np.float32(low)) / np.float32(high - low)
    normalized = np.clip((normalized - 0.5) * float(contrast) + 0.5, 0.0, 1.0)
    normalized = np.clip(normalized * float(brightness), 0.0, 1.0)
    source = np.rint(normalized * 255.0).astype(np.uint8)
    beams, bins = source.shape
    range_map, beam_map, valid, center_x, origin_y, max_radius = _fan_maps(
        int(width), int(height), int(beams), int(bins),
    )
    sampled = cv2.remap(
        source, range_map, beam_map, interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0,
    )
    rgb = np.empty((int(height), int(width), 3), dtype=np.uint8)
    rgb[:] = BACKGROUND
    rgb[valid] = _colour_lut()[sampled[valid]]
    image = Image.fromarray(rgb, mode="RGB")
    draw = ImageDraw.Draw(image)

    def point(angle: float, radius: float):
        return center_x + math.sin(angle) * radius, origin_y - math.cos(angle) * radius

    draw.arc(
        (center_x - max_radius, origin_y - max_radius, center_x + max_radius, origin_y + max_radius),
        50, 130, fill="#7e9aaa",
    )
    draw.line(point(math.radians(-40), max_radius) + point(math.radians(40), max_radius), fill="#7e9aaa")
    draw.line((center_x, origin_y, center_x, origin_y - max_radius), fill="#526b7c")
    start_m = float(record.get("range_start_m", 0.0))
    end_m = max(float(record.get("range_end_m", 10.0)), start_m + 1e-6)
    for distance in (end_m * 0.25, end_m * 0.5, end_m * 0.75, end_m):
        radius = max_radius * distance / end_m
        draw.ellipse((center_x - radius, origin_y - radius, center_x + radius, origin_y + radius), outline="#294352")
        draw.text((center_x + 5, origin_y - radius - 14), "%.1f m" % distance, fill="#b4c8d2")
    if show_atof:
        for point_data in record.get("points", []):
            angle = float(point_data.get("angle_rad", 0.0))
            distance = float(point_data.get("distance_m", 0.0))
            radius = max_radius * max(0.0, min(1.0, distance / end_m))
            x, y = point(angle, radius)
            draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill="#ffffff", outline="#101820")
    return image
