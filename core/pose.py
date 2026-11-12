import math
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional, Tuple

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from config import Field as FieldConfig
from config import PoseSensor as PoseSensorConfig


@dataclass
class PoseEstimate:
    """
    Represents the robot's estimated pose on the field.
    Coordinate system:
      - Origin (0, 0): Bottom-left corner of the field
      - X-axis: Left (0.0) to Right (WIDTH_CM = 400.0 cm)
      - Y-axis: Bottom (0.0) to Top (LENGTH_CM = 500.0 cm; 3x3 target grid is at Top)
      - heading_deg: 0.0 deg faces Top (+Y), 90.0 deg faces Right (+X)
    """

    x_cm: Optional[float]
    y_cm: Optional[float]
    heading_deg: float
    raw_heading_deg: float
    dist_front_cm: float
    dist_left_cm: Optional[float]
    dist_right_cm: float
    last_valid_left_cm: Optional[float]
    last_valid_right_cm: Optional[float]
    last_valid_front_cm: Optional[float]
    lr_status: str
    front_status: str
    valid: bool
    timestamp: float

    def to_dict(self) -> dict:
        return asdict(self)


class PoseEstimator:
    """
    Calculates robot field pose (x_cm, y_cm, heading_deg) from:
      - 2 directional LiDAR distances (front, right) in cm (or 3 if left is provided)
      - Geomagnetic heading angle in degrees

    Obstacle Rejection Strategy:
      - In 2-LiDAR mode (Front 0 deg + Right +90 deg):
          Independently tracks `last_valid_front_cm` and `last_valid_right_cm`.
          If either beam experiences a sudden single-step jump exceeding
          `max_front_delta_cm` / `max_right_delta_cm` (an obstacle entering the ray),
          that reading is rejected (`front_status="rejected_delta"` or
          `lr_status="right_rejected_delta"`) and the estimator holds the previous
          valid coordinate along that axis until the obstacle clears.
      - In 3-LiDAR mode (when `dist_left_cm` is also provided):
          Additionally cross-validates opposing Left and Right wall projections
          within `span_tolerance_cm`.
    """

    def __init__(
        self,
        field_width_cm: float = FieldConfig.WIDTH_CM,
        field_length_cm: float = FieldConfig.LENGTH_CM,
        front_offset_cm: Tuple[float, float] = (
            PoseSensorConfig.FRONT_OFFSET_X_CM,
            PoseSensorConfig.FRONT_OFFSET_Y_CM,
        ),
        left_offset_cm: Tuple[float, float] = (
            PoseSensorConfig.LEFT_OFFSET_X_CM,
            PoseSensorConfig.LEFT_OFFSET_Y_CM,
        ),
        right_offset_cm: Tuple[float, float] = (
            PoseSensorConfig.RIGHT_OFFSET_X_CM,
            PoseSensorConfig.RIGHT_OFFSET_Y_CM,
        ),
        clockwise_heading: bool = PoseSensorConfig.CLOCKWISE_HEADING,
        heading_offset_deg: float = PoseSensorConfig.DEFAULT_HEADING_OFFSET_DEG,
        min_range_cm: float = PoseSensorConfig.MIN_RANGE_CM,
        max_range_cm: float = PoseSensorConfig.MAX_RANGE_CM,
        span_tolerance_cm: float = PoseSensorConfig.SPAN_TOLERANCE_CM,
        max_ray_incidence_deg: float = PoseSensorConfig.MAX_RAY_INCIDENCE_DEG,
        max_front_delta_cm: float = PoseSensorConfig.MAX_FRONT_DELTA_CM,
        max_right_delta_cm: float = getattr(
            PoseSensorConfig, "MAX_RIGHT_DELTA_CM", PoseSensorConfig.MAX_FRONT_DELTA_CM
        ),
    ) -> None:
        self.field_width_cm = float(field_width_cm)
        self.field_length_cm = float(field_length_cm)

        # Sensor mounting offsets (dx_right, dy_front) in cm relative to robot center
        self.front_offset_cm = (float(front_offset_cm[0]), float(front_offset_cm[1]))
        self.left_offset_cm = (float(left_offset_cm[0]), float(left_offset_cm[1]))
        self.right_offset_cm = (float(right_offset_cm[0]), float(right_offset_cm[1]))

        self.clockwise_heading = clockwise_heading
        self.heading_offset_deg = float(heading_offset_deg)

        self.min_range_cm = float(min_range_cm)
        self.max_range_cm = float(max_range_cm)
        self.span_tolerance_cm = float(span_tolerance_cm)
        self.max_ray_incidence_deg = float(max_ray_incidence_deg)
        self.max_front_delta_cm = float(max_front_delta_cm)
        self.max_right_delta_cm = float(max_right_delta_cm)

        self._lock = threading.Lock()

        # Latest validated sensor distances (cm) for delta-based obstacle rejection
        self.last_valid_left_cm: Optional[float] = None
        self.last_valid_right_cm: Optional[float] = None
        self.last_valid_front_cm: Optional[float] = None

        # Last known valid field coordinates (cm)
        self._last_x_cm: Optional[float] = None
        self._last_y_cm: Optional[float] = None
        self._latest_pose: Optional[PoseEstimate] = None

    # --------------------------------------------------------------------------
    # Calibration & State Reset
    # --------------------------------------------------------------------------
    def calibrate_heading(
        self, raw_heading_deg: float, target_field_deg: float = 0.0
    ) -> float:
        """
        Sets `heading_offset_deg` so that `raw_heading_deg` maps to `target_field_deg`
        (default 0.0 deg = facing Top wall / 3x3 grid).
        """
        with self._lock:
            signed_raw = raw_heading_deg if self.clockwise_heading else -raw_heading_deg
            self.heading_offset_deg = (signed_raw - target_field_deg) % 360.0
            return self.heading_offset_deg

    def reset_history(self) -> None:
        """Clears cached valid distances and pose history."""
        with self._lock:
            self.last_valid_left_cm = None
            self.last_valid_right_cm = None
            self.last_valid_front_cm = None
            self._last_x_cm = None
            self._last_y_cm = None
            self._latest_pose = None

    @property
    def latest_pose(self) -> Optional[PoseEstimate]:
        with self._lock:
            return self._latest_pose

    # --------------------------------------------------------------------------
    # Geometry Helpers
    # --------------------------------------------------------------------------
    def normalize_heading(self, raw_heading_deg: float) -> float:
        """
        Converts raw geomagnetic heading into field heading [0, 360) deg
        where 0 deg = Top (+Y), 90 deg = Right (+X), 180 deg = Bottom (-Y), 270 deg = Left (-X).
        """
        signed_raw = raw_heading_deg if self.clockwise_heading else -raw_heading_deg
        return (signed_raw - self.heading_offset_deg) % 360.0

    def _is_range_valid(self, dist_cm: Optional[float]) -> bool:
        return (
            dist_cm is not None
            and math.isfinite(dist_cm)
            and self.min_range_cm <= dist_cm <= self.max_range_cm
        )

    def _project_ray_to_wall(
        self,
        dist_cm: Optional[float],
        heading_deg: float,
        rel_beam_deg: float,
        offset_xy_cm: Tuple[float, float],
    ) -> Tuple[Optional[str], Optional[float]]:
        """
        Projects a single LiDAR ray onto the field boundary walls.

        Returns:
          (axis, coord_cm) where `axis` is "x", "y", or None (if out of range or
          striking near a corner at > max_ray_incidence_deg), and `coord_cm` is
          the implied robot center coordinate along that axis.
        """
        if dist_cm is None or not self._is_range_valid(dist_cm):
            return None, None

        theta_rad = math.radians(heading_deg)
        alpha_deg = (heading_deg + rel_beam_deg) % 360.0
        alpha_rad = math.radians(alpha_deg)

        dx_body, dy_body = offset_xy_cm
        # Sensor mounting offset rotated into field frame (+X right, +Y top)
        sensor_shift_x = dx_body * math.cos(theta_rad) + dy_body * math.sin(theta_rad)
        sensor_shift_y = -dx_body * math.sin(theta_rad) + dy_body * math.cos(theta_rad)

        # Ray unit vector in field frame: (sin(alpha), cos(alpha))
        dir_x = math.sin(alpha_rad)
        dir_y = math.cos(alpha_rad)

        # Total displacement from robot center to the beam hit point
        hit_dx = sensor_shift_x + dist_cm * dir_x
        hit_dy = sensor_shift_y + dist_cm * dir_y

        min_cos = math.cos(math.radians(self.max_ray_incidence_deg))
        margin = self.span_tolerance_cm

        # Check if ray points predominantly toward Left (-X) or Right (+X) wall
        if abs(dir_x) >= min_cos:
            if dir_x < 0:
                # Ray hits Left wall (x_wall = 0) -> 0 = x_robot + hit_dx
                x_est = -hit_dx
            else:
                # Ray hits Right wall (x_wall = W) -> W = x_robot + hit_dx
                x_est = self.field_width_cm - hit_dx

            if -margin <= x_est <= (self.field_width_cm + margin):
                x_clamped = max(0.0, min(self.field_width_cm, x_est))
                return "x", x_clamped
            return None, None

        # Check if ray points predominantly toward Bottom (-Y) or Top (+Y) wall
        if abs(dir_y) >= min_cos:
            if dir_y > 0:
                # Ray hits Top wall (y_wall = L) -> L = y_robot + hit_dy
                y_est = self.field_length_cm - hit_dy
            else:
                # Ray hits Bottom wall (y_wall = 0) -> 0 = y_robot + hit_dy
                y_est = -hit_dy

            if -margin <= y_est <= (self.field_length_cm + margin):
                y_clamped = max(0.0, min(self.field_length_cm, y_est))
                return "y", y_clamped
            return None, None

        return None, None

    # --------------------------------------------------------------------------
    # Main Pose Update with Delta-Based Obstacle Rejection
    # --------------------------------------------------------------------------
    def update_from_mm(
        self,
        dist_front_mm: int,
        dist_right_mm: int,
        dist_left_mm: int,
        raw_heading_deg: float,
    ) -> PoseEstimate:
        """
        Convenience wrapper accepting ESP32 3-LiDAR measurements in millimeters (uint32)
        and converting them to centimeters before running pose estimation.
        """
        front_cm = (
            float(dist_front_mm) / 10.0
            if 0 < int(dist_front_mm) < 0xFFFFFFFF
            else -1.0
        )
        right_cm = (
            float(dist_right_mm) / 10.0
            if 0 < int(dist_right_mm) < 0xFFFFFFFF
            else -1.0
        )
        left_cm = (
            float(dist_left_mm) / 10.0
            if dist_left_mm is not None and 0 < int(dist_left_mm) < 0xFFFFFFFF
            else None
        )
        return self.update(
            dist_front_cm=front_cm,
            dist_right_cm=right_cm,
            raw_heading_deg=raw_heading_deg,
            dist_left_cm=left_cm,
        )

    def update(
        self,
        dist_front_cm: float,
        dist_right_cm: float,
        raw_heading_deg: float,
        dist_left_cm: Optional[float] = None,
    ) -> PoseEstimate:
        """
        Computes a new field pose estimate from Front & Right LiDAR distances (in cm),
        geomagnetic heading (in degrees), and optional Left LiDAR distance (in cm).
        """
        with self._lock:
            heading_deg = self.normalize_heading(raw_heading_deg)

            # 1. Project Right (+90 deg) and optional Left (-90 deg) beams
            axis_l, pos_l = (
                self._project_ray_to_wall(
                    dist_left_cm, heading_deg, -90.0, self.left_offset_cm
                )
                if dist_left_cm is not None
                else (None, None)
            )
            axis_r, pos_r = self._project_ray_to_wall(
                dist_right_cm, heading_deg, +90.0, self.right_offset_cm
            )

            lr_axis: Optional[str] = axis_r or axis_l
            lr_pos: Optional[float] = None
            lr_status = "no_valid_beam"

            if axis_l is not None and axis_r is not None and axis_l == axis_r:
                assert pos_l is not None and pos_r is not None and dist_left_cm is not None
                discrepancy = abs(pos_l - pos_r)

                if discrepancy <= self.span_tolerance_cm:
                    lr_pos = 0.5 * (pos_l + pos_r)
                    self.last_valid_left_cm = dist_left_cm
                    self.last_valid_right_cm = dist_right_cm
                    lr_status = "both_valid"
                else:
                    if (
                        self.last_valid_left_cm is not None
                        and self.last_valid_right_cm is not None
                    ):
                        delta_l = abs(dist_left_cm - self.last_valid_left_cm)
                        delta_r = abs(dist_right_cm - self.last_valid_right_cm)

                        if delta_l <= delta_r:
                            lr_pos = pos_l
                            self.last_valid_left_cm = dist_left_cm
                            lr_status = "left_only_rejected_right"
                        else:
                            lr_pos = pos_r
                            self.last_valid_right_cm = dist_right_cm
                            lr_status = "right_only_rejected_left"
                    elif self.last_valid_left_cm is not None:
                        delta_l = abs(dist_left_cm - self.last_valid_left_cm)
                        if delta_l <= self.max_front_delta_cm:
                            lr_pos = pos_l
                            self.last_valid_left_cm = dist_left_cm
                            lr_status = "left_only_rejected_right"
                        else:
                            lr_pos = pos_r
                            self.last_valid_right_cm = dist_right_cm
                            lr_status = "right_only_rejected_left"
                    elif self.last_valid_right_cm is not None:
                        delta_r = abs(dist_right_cm - self.last_valid_right_cm)
                        if delta_r <= self.max_right_delta_cm:
                            lr_pos = pos_r
                            self.last_valid_right_cm = dist_right_cm
                            lr_status = "right_only_rejected_left"
                        else:
                            lr_pos = pos_l
                            self.last_valid_left_cm = dist_left_cm
                            lr_status = "left_only_rejected_right"
                    else:
                        if dist_left_cm >= dist_right_cm:
                            lr_pos = pos_l
                            self.last_valid_left_cm = dist_left_cm
                            lr_status = "left_only_bootstrap"
                        else:
                            lr_pos = pos_r
                            self.last_valid_right_cm = dist_right_cm
                            lr_status = "right_only_bootstrap"

            elif axis_r is not None and pos_r is not None:
                # 2-LiDAR default mode (Right beam only) or Left beam invalid
                if (
                    self.last_valid_right_cm is None
                    or abs(dist_right_cm - self.last_valid_right_cm) <= self.max_right_delta_cm
                ):
                    lr_pos = pos_r
                    self.last_valid_right_cm = dist_right_cm
                    lr_status = "right_only"
                else:
                    lr_status = "right_rejected_delta"

            elif axis_l is not None and pos_l is not None and dist_left_cm is not None:
                # Only Left beam produced a valid wall projection
                if (
                    self.last_valid_left_cm is None
                    or abs(dist_left_cm - self.last_valid_left_cm) <= self.max_front_delta_cm
                ):
                    lr_pos = pos_l
                    self.last_valid_left_cm = dist_left_cm
                    lr_status = "left_only"
                else:
                    lr_status = "left_rejected_delta"

            # 2. Project Front (0 deg) beam
            axis_f, pos_f = self._project_ray_to_wall(
                dist_front_cm, heading_deg, 0.0, self.front_offset_cm
            )
            front_pos: Optional[float] = None
            front_status = "no_valid_beam"

            if axis_f is not None and pos_f is not None:
                if (
                    self.last_valid_front_cm is None
                    or abs(dist_front_cm - self.last_valid_front_cm) <= self.max_front_delta_cm
                ):
                    front_pos = pos_f
                    self.last_valid_front_cm = dist_front_cm
                    front_status = "valid"
                else:
                    front_status = "rejected_delta"

            # 3. Assign resolved axes to (x_cm, y_cm), falling back to last valid coordinate if needed
            x_cm = self._last_x_cm
            y_cm = self._last_y_cm

            if lr_axis == "x" and lr_pos is not None:
                x_cm = round(lr_pos, 2)
            elif lr_axis == "y" and lr_pos is not None:
                y_cm = round(lr_pos, 2)

            if axis_f == "y" and front_pos is not None:
                y_cm = round(front_pos, 2)
            elif axis_f == "x" and front_pos is not None:
                x_cm = round(front_pos, 2)

            self._last_x_cm = x_cm
            self._last_y_cm = y_cm

            estimate = PoseEstimate(
                x_cm=x_cm,
                y_cm=y_cm,
                heading_deg=round(heading_deg, 2),
                raw_heading_deg=round(raw_heading_deg, 2),
                dist_front_cm=round(dist_front_cm, 2),
                dist_left_cm=round(dist_left_cm, 2) if dist_left_cm is not None else None,
                dist_right_cm=round(dist_right_cm, 2),
                last_valid_left_cm=(
                    round(self.last_valid_left_cm, 2)
                    if self.last_valid_left_cm is not None
                    else None
                ),
                last_valid_right_cm=(
                    round(self.last_valid_right_cm, 2)
                    if self.last_valid_right_cm is not None
                    else None
                ),
                last_valid_front_cm=(
                    round(self.last_valid_front_cm, 2)
                    if self.last_valid_front_cm is not None
                    else None
                ),
                lr_status=lr_status,
                front_status=front_status,
                valid=(x_cm is not None and y_cm is not None),
                timestamp=time.time(),
            )
            self._latest_pose = estimate
            return estimate


# ==============================================================================
# Unit Verification
# ==============================================================================
if __name__ == "__main__":
    print("Testing PoseEstimator (3-LiDAR Front + Right + Left mode) on 400cm (W) x 500cm (L) field...")
    est = PoseEstimator()

    # 1. Robot at (x=150, y=120), facing Top wall (0 deg):
    #    dist_front = 3800 mm (380 cm)
    #    dist_right = 2500 mm (250 cm)
    #    dist_left  = 1500 mm (150 cm)
    p1 = est.update_from_mm(
        dist_front_mm=3800, dist_right_mm=2500, dist_left_mm=1500, raw_heading_deg=0.0
    )
    assert p1.valid and p1.x_cm == 150.0 and p1.y_cm == 120.0
    assert p1.lr_status == "both_valid" and p1.front_status == "valid"
    print("1. 3-LiDAR normal pose passed:", p1)

    # 2. Robot moves to (x=154, y=125), but an obstacle blocks Right LiDAR at 550 mm (55 cm)!
    #    dist_left = 1540 mm (154 cm, delta=4 cm), dist_right = 550 mm (55 cm, delta=195 cm)
    #    Should reject Right and trust Left -> x = 154.0!
    p2 = est.update_from_mm(
        dist_front_mm=3750, dist_right_mm=550, dist_left_mm=1540, raw_heading_deg=0.0
    )
    assert p2.x_cm == 154.0 and p2.y_cm == 125.0
    assert p2.lr_status == "left_only_rejected_right"
    print("2. 3-LiDAR Right obstacle rejection passed:", p2.lr_status, f"x={p2.x_cm}, y={p2.y_cm}")

    # 3. Obstacle clears on Right (dist_left=1560 mm, dist_right=2440 mm -> sum=400 cm)
    p3 = est.update_from_mm(
        dist_front_mm=3720, dist_right_mm=2440, dist_left_mm=1560, raw_heading_deg=0.0
    )
    assert p3.lr_status == "both_valid" and p3.x_cm == 156.0 and p3.y_cm == 128.0
    print("3. 3-LiDAR Right recovery passed:", p3.lr_status, f"x={p3.x_cm}, y={p3.y_cm}")
    print("All PoseEstimator tests passed!")
