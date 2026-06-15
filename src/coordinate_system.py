"""
UAV Coordinate System Module — Pixel-to-GPS Mapping & Terrain Analysis Framework
================================================================================
Version: v2.0 (2026-06-14)

A general-purpose coordinate system library for UAV simulation, providing:
  1. Fence/Region Definition (GPS polygon)
  2. Drone Pose Acquisition (AirSim GPS + IMU)
  3. Homography Matrix Calibration (pixel ↔ ground)
  4. Pixel → World GPS Coordinate Conversion (Pinhole & Homography)
  5. Terrain Object Detection Mapping & Spatial Analysis
  6. Real-time 3D Visualization (GPU-accelerated)

Coordinate Pipeline (Method A - Pinhole Model):
  (u, v) Pixel Coord
    → Pinhole Camera Model (FOV/Resolution based)
    → Ground Offset → Yaw Rotation → GPS Coordinate

Coordinate Pipeline (Method B - Homography, Recommended):
  (u, v) Pixel Coord
    → Homography Matrix (calibrated from fence corners)
    → Ground Coord (m) → GPS Coordinate

Dependencies:
  - airsim (AirSim Python API)
  - numpy, cv2, scipy
  - sklearn (DBSCAN clustering for spatial analysis)
"""
import numpy as np
import math
import cv2
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any


# =============================================================================
# 1. Fence / Region Definition
# =============================================================================
@dataclass
class FenceConfig:
    """Fence / region-of-interest defined by GPS coordinates of boundary corners."""
    corners_latlon: List[Tuple[float, float]] = field(default_factory=lambda: [
        (47.641468, -122.140165),   # SW corner
        (47.642100, -122.140165),   # NW corner
        (47.642100, -122.139165),   # NE corner
        (47.641468, -122.139165),   # SE corner
    ])

    def as_polygon(self) -> np.ndarray:
        """Returns N×2 array [latitude, longitude]."""
        return np.array(self.corners_latlon)

    def center(self) -> Tuple[float, float]:
        """Compute fence center (latitude, longitude)."""
        pts = self.as_polygon()
        return float(pts[:, 0].mean()), float(pts[:, 1].mean())

    def bounds_meters(self, ref_lat: float, ref_lon: float) -> Tuple[float, float, float, float]:
        """
        Compute fence ENU bounds relative to a reference point.
        Returns: (min_east, max_east, min_north, max_north)
        """
        enu_pts = np.array([latlon_to_enu(lat, lon, ref_lat, ref_lon)
                           for lat, lon in self.corners_latlon])
        return (float(enu_pts[:, 0].min()), float(enu_pts[:, 0].max()),
                float(enu_pts[:, 1].min()), float(enu_pts[:, 1].max()))

    def __post_init__(self):
        assert len(self.corners_latlon) >= 3, "Fence needs >=3 vertices"

    def get_corner_gps(self) -> List[Tuple[float, float]]:
        """Return fence corner GPS list (for Homography calibration)."""
        return self.corners_latlon


# =============================================================================
# 2. GPS / Coordinate Conversion Utilities
# =============================================================================
_EARTH_RADIUS = 6371000  # WGS-84 Earth radius (m)


def gps_to_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float]:
    """
    Compute approximate relative displacement (m) between two GPS points.
    Returns: (dx_east, dy_north) — east and north offset from point1 to point2.
    """
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    dy = dlat * _EARTH_RADIUS
    dx = dlon * _EARTH_RADIUS * math.cos(math.radians((lat1 + lat2) / 2))
    return dx, dy


def latlon_to_enu(lat: float, lon: float,
                  ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    GPS → ENU coordinates (east, north in meters).
    """
    dx, dy = gps_to_meters(ref_lat, ref_lon, lat, lon)
    return dx, dy


def enu_to_latlon(east: float, north: float,
                  ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    ENU coordinates (east, north in meters) → GPS (lat, lon).
    """
    dlat = north / _EARTH_RADIUS
    dlon = east / (_EARTH_RADIUS * math.cos(math.radians(ref_lat)))
    return ref_lat + math.degrees(dlat), ref_lon + math.degrees(dlon)


def distance_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two GPS points (meters)."""
    dx, dy = gps_to_meters(lat1, lon1, lat2, lon2)
    return math.sqrt(dx**2 + dy**2)


# =============================================================================
# 3. Camera Model: Pixel → Camera Ray (Method A - Pinhole Model)
# =============================================================================
class BottomCamera:
    """
    Downward-facing bottom camera pinhole model.
    Converts pixel coordinates (u, v) into direction vectors in camera space.

    Suitable for:
      - AirSim simulation (known FOV / resolution)
      - Rapid prototyping in simulated scenarios
    """
    def __init__(self,
                 img_width: int = 640,
                 img_height: int = 640,
                 fov_degrees: float = 120.0):
        self.width = img_width
        self.height = img_height
        self.fov = math.radians(fov_degrees)

        # Compute intrinsics K
        self.fx = (img_width / 2) / math.tan(self.fov / 2)
        self.fy = (img_height / 2) / math.tan(self.fov / 2)
        self.cx = img_width / 2
        self.cy = img_height / 2

        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])

    def pixel_to_camera(self, u: float, v: float, depth: float = 1.0) -> np.ndarray:
        """Pixel coord → 3D point in camera coordinates (right-X, down-Y, forward-Z)."""
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth
        return np.array([x, y, z])

    def camera_to_pixel(self, x: float, y: float, z: float) -> Tuple[float, float]:
        """Camera coordinates → pixel coord."""
        u = self.fx * x / z + self.cx
        v = self.fy * y / z + self.cy
        return u, v

    def pixel_ray(self, u: float, v: float) -> np.ndarray:
        """Normalized ray direction (camera coordinates) for a given pixel."""
        vec = self.pixel_to_camera(u, v, depth=1.0)
        return vec / np.linalg.norm(vec)

    def ground_projection(self, u: float, v: float,
                          altitude: float) -> np.ndarray:
        """
        Project pixel coord onto the ground plane (assuming level terrain, camera looking down).
        Returns: (x_east, y_north) ground offset in meters.
        """
        cam_pt = self.pixel_to_camera(u, v, depth=altitude)
        return np.array([cam_pt[0], cam_pt[2]])

    def get_intrinsics_dict(self) -> Dict[str, Any]:
        """Return camera intrinsics dictionary."""
        return {
            'fx': self.fx, 'fy': self.fy,
            'cx': self.cx, 'cy': self.cy,
            'width': self.width, 'height': self.height,
            'fov_deg': math.degrees(self.fov)
        }


# =============================================================================
# 4. Homography Mapping (Method B - Recommended)
# =============================================================================
class HomographyMapper:
    """
    Homography-based pixel-to-ground mapping.

    Principle:
      Uses the known GPS coordinates of fence corners and their corresponding
      pixel coordinates in the camera image to solve for the homography matrix H (3×3).
      This enables direct pixel → ground → GPS mapping.

    Advantages:
      - No camera intrinsics required
      - Single calibration sufficient
      - Naturally corrects perspective distortion
      - Real-world deployment can use ArUco/AprilTag auto-calibration

    Usage:
      Method A (Sim): Project known fence GPS through pinhole model → pixel coords → solve H
      Method B (Real): ArUco markers at fence corners → detect in image → solve H
    """
    def __init__(self, fence: 'FenceConfig', camera: 'BottomCamera',
                 drone_altitude: float = 30.0):
        """
        Initialize Homography mapper.

        Args:
            fence: Fence configuration (4+ GPS corner coordinates)
            camera: Camera model (used for pixel projection in calibration)
            drone_altitude: Drone flight altitude (m), used for calibration
        """
        self.fence = fence
        self.camera = camera
        self.altitude = drone_altitude

        # Ground coord origin = fence center
        self.ref_lat, self.ref_lon = fence.center()

        # Build homography
        self._calibrate(drone_altitude)

        # Store previous H for comparison
        self.last_H = self.H.copy()

    def _calibrate(self, altitude: float):
        """
        Calibrate the homography matrix.

        Steps:
          1. Fence corner GPS → ENU coordinates (ground coord, m)
          2. Fence corner GPS → pixel coordinates via camera model
          3. Solve homography: pixel ↔ ground
        """
        fence_gps = self.fence.get_corner_gps()

        # 1. Ground coordinates: GPS → ENU (fence center as origin)
        self.ground_pts = np.array([
            latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon)
            for lat, lon in fence_gps
        ], dtype=np.float32)

        # 2. Pixel coordinates: project corners through the camera model
        pixel_pts = []
        for lat, lon in fence_gps:
            e_offset, n_offset = latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon)
            # Assuming camera boresight is aligned with fence center
            u = self.camera.cx + e_offset * self.camera.fx / altitude
            v = self.camera.cy - n_offset * self.camera.fy / altitude  # image v-axis points down
            pixel_pts.append([u, v])

        self.pixel_pts = np.array(pixel_pts, dtype=np.float32)

        # 3. Solve homography: pixel ↔ ground
        self.H, _ = cv2.findHomography(self.pixel_pts, self.ground_pts)
        if self.H is None:
            # Fallback: scaled approximation
            print("[Homography] Warning: findHomography failed, using approximate matrix")
            scale = altitude / self.camera.fx
            self.H = np.array([
                [scale, 0, -scale * self.camera.cx],
                [0, -scale, scale * self.camera.cy],
                [0, 0, 1]
            ], dtype=np.float32)

        self.H_inv = np.linalg.inv(self.H)

    def pixel_to_ground(self, u: float, v: float) -> Tuple[float, float]:
        """
        Pixel coord → ground ENU coordinates (meters).

        Args:
            u, v: Pixel coordinates

        Returns:
            (east, north) relative to fence center
        """
        pt = np.array([[[u, v]]], dtype=np.float32)
        ground = cv2.perspectiveTransform(pt, self.H)
        return float(ground[0, 0, 0]), float(ground[0, 0, 1])

    def pixel_to_gps(self, u: float, v: float) -> Tuple[float, float]:
        """
        Pixel coord → GPS coordinates.

        Returns:
            (latitude, longitude)
        """
        east, north = self.pixel_to_ground(u, v)
        return enu_to_latlon(east, north, self.ref_lat, self.ref_lon)

    def ground_to_pixel(self, east: float, north: float) -> Tuple[float, float]:
        """Ground ENU coordinates → pixel coord (inverse mapping)."""
        pt = np.array([[[east, north]]], dtype=np.float32)
        pixel = cv2.perspectiveTransform(pt, self.H_inv)
        return float(pixel[0, 0, 0]), float(pixel[0, 0, 1])

    def recalibrate(self, altitude: float = None):
        """Recalibrate when drone altitude changes."""
        if altitude is not None:
            self.altitude = altitude
        self.last_H = self.H.copy()
        self._calibrate(self.altitude)

    def get_fence_in_image(self, img_width: int, img_height: int) -> np.ndarray:
        """Project fence corners into image pixel coords (for visualization)."""
        fence_pixels = []
        for lat, lon in self.fence.get_corner_gps():
            e, n = latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon)
            u, v = self.ground_to_pixel(e, n)
            fence_pixels.append([u, v])
        return np.array(fence_pixels, dtype=np.int32)

    def get_homography_matrix(self) -> np.ndarray:
        """Return the homography matrix (3×3)."""
        return self.H.copy()


# =============================================================================
# 5. Spatial Analysis: SpatialAnalyzer
# =============================================================================
class SpatialAnalyzer:
    """
    Spatial analyzer for detected objects in UAV imagery.

    Features:
      - Weighted center computation (confidence × area)
      - HDBSCAN clustering for automatic group discovery
      - Alpha Shape boundary detection (concave hull)
      - Dispersion analysis and nearest-neighbor statistics
      - Outlier detection

    Uses HDBSCAN (no preset cluster count needed) + Alpha Shape
    for accurate spatial grouping even with irregular distributions.
    """
    from scipy.spatial import ConvexHull as _ConvexHull

    def __init__(self, ref_lat: float = None, ref_lon: float = None,
                 alpha: float = None):
        """
        Args:
            ref_lat, ref_lon: Reference point for ENU conversion
            alpha: Alpha shape parameter (None=auto)
        """
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.alpha = alpha

    def analyze(self, detections: List[Tuple]) -> Dict[str, Any]:
        """
        Analyze spatial distribution of detected objects.

        HDBSCAN automatically finds optimal clusters and marks noise as outliers.
        Alpha Shape produces a concave hull that follows the actual shape.

        Args:
            detections: [(lat, lon, confidence, box_w, box_h), ...]

        Returns:
            {
                'center': (lat, lon),           # Weighted center
                'center_enu': (east, north),
                'clusters': [...],              # Per-cluster info
                'outliers': [(lat,lon), ...],
                'n_objects': int,
                'n_clusters': int,
                'hull': [(lat,lon), ...],
                'hull_area_m2': float,
                'dispersion': float,
                'avg_neighbor_dist': float,
            }
        """
        import hdbscan
        import alphashape

        n_objects = len(detections)
        if n_objects == 0:
            return {
                'center': (0, 0), 'center_enu': (0, 0),
                'clusters': [], 'outliers': [],
                'n_objects': 0, 'n_clusters': 0,
                'hull': [], 'hull_area_m2': 0,
                'dispersion': 0, 'avg_neighbor_dist': 0,
            }

        # Parse input
        parsed = []
        for det in detections:
            if len(det) >= 5:
                lat, lon, conf, w, h = det[:5]
                area = w * h
            elif len(det) >= 3:
                lat, lon, conf = det[:3]
                area = 1.0
            else:
                continue
            parsed.append({'lat': lat, 'lon': lon,
                          'conf': conf, 'area': area})

        if self.ref_lat is None:
            self.ref_lat = sum(p['lat'] for p in parsed) / len(parsed)
        if self.ref_lon is None:
            self.ref_lon = sum(p['lon'] for p in parsed) / len(parsed)

        # Convert to ENU
        positions_enu = []
        for p in parsed:
            e, n = latlon_to_enu(p['lat'], p['lon'],
                                 self.ref_lat, self.ref_lon)
            positions_enu.append([e, n, p['conf'], p['area']])
        positions_enu = np.array(positions_enu)
        coords = positions_enu[:, :2]
        weights = positions_enu[:, 2] * positions_enu[:, 3]

        # ---- (1) Weighted center ----
        if weights.sum() > 0:
            center_e = (coords[:, 0] * weights).sum() / weights.sum()
            center_n = (coords[:, 1] * weights).sum() / weights.sum()
        else:
            center_e, center_n = coords[:, 0].mean(), coords[:, 1].mean()
        center_lat, center_lon = enu_to_latlon(
            center_e, center_n, self.ref_lat, self.ref_lon)

        # ---- (2) HDBSCAN clustering ----
        n = len(coords)
        if n <= 4:
            cluster_labels = np.zeros(n, dtype=int)
        else:
            min_cluster_size = max(2, n // 3)
            clusterer = hdbscan.HDBSCAN(
                min_cluster_size=min_cluster_size,
                min_samples=1,
                metric='euclidean',
                cluster_selection_method='leaf'
            )
            cluster_labels = clusterer.fit_predict(coords)

            noise_ratio = np.mean(cluster_labels == -1)
            if noise_ratio > 0.8:
                cluster_labels = np.zeros(n, dtype=int)

        # Separate outliers from clustered objects
        is_noise = cluster_labels == -1
        outlier_coords_list = [tuple(c) for c in coords[is_noise]]
        core_mask = ~is_noise
        core_coords = coords[core_mask]
        core_labels = cluster_labels[core_mask]
        core_indices = np.where(core_mask)[0]
        n_core = len(core_coords)
        k_used = len(set(core_labels)) if n_core > 0 else 0

        # ---- (3) Build clusters ----
        clusters = []
        boundary_coords_list = []

        for label in set(core_labels):
            mask = core_labels == label
            idx_in_core = np.where(mask)[0]
            idx_full = core_indices[idx_in_core]

            cluster_pts = positions_enu[idx_full]
            cluster_coords = cluster_pts[:, :2]

            c_weights = cluster_pts[:, 2] * cluster_pts[:, 3]
            if c_weights.sum() > 0:
                c_e = (cluster_coords[:, 0] * c_weights).sum() / c_weights.sum()
                c_n = (cluster_coords[:, 1] * c_weights).sum() / c_weights.sum()
            else:
                c_e, c_n = cluster_coords[:, 0].mean(), cluster_coords[:, 1].mean()
            c_lat, c_lon = enu_to_latlon(c_e, c_n, self.ref_lat, self.ref_lon)

            hull_area = 0.0
            if len(cluster_coords) >= 3:
                c_hull = self._ConvexHull(cluster_coords)
                hull_area = c_hull.volume

            objects_in_cluster = [
                enu_to_latlon(float(p[0]), float(p[1]),
                              self.ref_lat, self.ref_lon)
                for p in cluster_pts
            ]

            clusters.append({
                'positions_enu': cluster_coords,
                'positions_gps': objects_in_cluster,
                'center': (c_lat, c_lon),
                'center_enu': (c_e, c_n),
                'count': int(mask.sum()),
                'area_m2': float(hull_area),
            })

            # Boundary points via Alpha Shape
            if len(cluster_coords) >= 4:
                try:
                    alpha_shape = alphashape.alphashape(
                        cluster_coords, alpha=self.alpha
                    )
                    if hasattr(alpha_shape, 'exterior'):
                        boundary_coords = np.array(alpha_shape.exterior.coords)
                        for bx, by in boundary_coords:
                            dists = np.sqrt(
                                (cluster_coords[:, 0] - bx)**2 +
                                (cluster_coords[:, 1] - by)**2
                            )
                            nearest = np.argmin(dists)
                            if dists[nearest] < 5.0:
                                boundary_coords_list.append((
                                    float(cluster_coords[nearest, 0]),
                                    float(cluster_coords[nearest, 1])
                                ))
                except Exception:
                    c_hull = self._ConvexHull(cluster_coords)
                    for idx in c_hull.vertices:
                        boundary_coords_list.append((
                            float(cluster_coords[idx, 0]),
                            float(cluster_coords[idx, 1])
                        ))
            elif len(cluster_coords) >= 3:
                c_hull = self._ConvexHull(cluster_coords)
                for idx in c_hull.vertices:
                    boundary_coords_list.append((
                        float(cluster_coords[idx, 0]),
                        float(cluster_coords[idx, 1])
                    ))

        clusters.sort(key=lambda c: c['count'], reverse=True)
        boundary_coords_list = list(set(boundary_coords_list))

        outliers_gps = [
            enu_to_latlon(e, n, self.ref_lat, self.ref_lon)
            for e, n in outlier_coords_list
        ]
        boundary_gps = [
            enu_to_latlon(e, n, self.ref_lat, self.ref_lon)
            for e, n in boundary_coords_list
        ]

        # ---- (4) Overall boundary (Alpha Shape) ----
        hull_gps = []
        hull_area_m2 = 0.0
        if len(core_coords) >= 3:
            try:
                alpha_shape = alphashape.alphashape(
                    core_coords, alpha=self.alpha
                )
                if hasattr(alpha_shape, 'exterior'):
                    hull_coords = np.array(alpha_shape.exterior.coords)
                    hull_area_m2 = alpha_shape.area
                    for x, y in hull_coords:
                        lat, lon = enu_to_latlon(
                            float(x), float(y),
                            self.ref_lat, self.ref_lon)
                        hull_gps.append((lat, lon))
            except Exception:
                hull = self._ConvexHull(core_coords)
                hull_area_m2 = hull.volume
                for idx in hull.vertices:
                    lat, lon = enu_to_latlon(
                        float(core_coords[idx, 0]), float(core_coords[idx, 1]),
                        self.ref_lat, self.ref_lon)
                    hull_gps.append((lat, lon))

        # ---- (5) Dispersion ----
        dispersion = 0.0
        avg_nn_dist = 0.0
        if len(core_coords) > 1:
            dists_center = np.sqrt(
                (core_coords[:, 0] - center_e)**2 +
                (core_coords[:, 1] - center_n)**2
            )
            dispersion = float(dists_center.std())
            nn_dists = []
            for i in range(len(core_coords)):
                other_dists = np.sqrt(
                    (core_coords[:, 0] - core_coords[i, 0])**2 +
                    (core_coords[:, 1] - core_coords[i, 1])**2
                )
                other_dists[i] = np.inf
                nn_dists.append(other_dists.min())
            avg_nn_dist = float(np.mean(nn_dists))

        return {
            'center': (center_lat, center_lon),
            'center_enu': (center_e, center_n),
            'clusters': clusters,
            'outliers': outliers_gps,
            'boundary_points': boundary_gps,
            'n_objects': len(parsed),
            'n_clusters': k_used,
            'hull': hull_gps,
            'hull_area_m2': hull_area_m2,
            'dispersion': dispersion,
            'avg_neighbor_dist': avg_nn_dist,
        }


# =============================================================================
# 6. Pose Transformation Utilities
# =============================================================================
def euler_to_rotation(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """
    Euler angles → rotation matrix (ZYX order: yaw, pitch, roll).
    Returns 3×3 rotation matrix transforming body-frame to world-frame.
    """
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)

    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr          ]
    ])


def body_to_world(body_pos: np.ndarray,
                  drone_lat: float, drone_lon: float, drone_alt: float,
                  yaw: float, pitch: float = 0, roll: float = 0) -> Tuple[float, float]:
    """
    Transform body-frame coordinates (north, east, down) to world GPS.
    """
    vec_body = np.array([body_pos[1], body_pos[0], -body_pos[2]])
    R = euler_to_rotation(yaw, pitch, roll)
    vec_world = R @ vec_body

    north = vec_world[0]
    east = vec_world[1]

    target_lat, target_lon = enu_to_latlon(east, north, drone_lat, drone_lon)
    return target_lat, target_lon


# =============================================================================
# 7. AirSim Integration: Drone Pose Provider
# =============================================================================
class AirsimPoseProvider:
    """Real-time drone pose provider from AirSim."""
    def __init__(self):
        import airsim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.home_lat = None
        self.home_lon = None

    def get_drone_pose(self) -> dict:
        """Get current drone position and orientation."""
        gps = self.client.getGpsData()
        imu = self.client.getImuData()
        q = imu.orientation
        sin_yaw = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
        cos_yaw = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
        return {
            'lat': gps.gnss.geo_point.latitude,
            'lon': gps.gnss.geo_point.longitude,
            'alt': gps.gnss.geo_point.altitude,
            'yaw': math.atan2(sin_yaw, cos_yaw),
        }

    def get_drone_pose_altitude(self) -> float:
        """Get drone altitude AGL (m)."""
        pose = self.client.simGetVehiclePose()
        return pose.position.z

    def set_home(self):
        """Record current position as home."""
        pose = self.get_drone_pose()
        self.home_lat = pose['lat']
        self.home_lon = pose['lon']
        print(f"[Coordinate] Home: ({self.home_lat:.6f}, {self.home_lon:.6f})")

    def get_altitude_above_ground(self) -> float:
        """Get drone height above ground (m)."""
        state = self.client.getMultirotorState()
        return abs(state.kinematics_estimated.position.z_val)


# =============================================================================
# 8. Detection → World Coordinate Mapping (backward compatible)
# =============================================================================
def detection_to_world(
    bbox_center: Tuple[float, float],
    drone_lat: float, drone_lon: float,
    altitude: float,
    yaw: float,
    camera: BottomCamera,
    pitch: float = 0, roll: float = 0
) -> Tuple[float, float]:
    """
    Convert detected object pixel position → world GPS coordinates.
    Uses Method A (pinhole model + ground projection).

    Returns: (latitude, longitude)
    """
    u, v = bbox_center
    ground_offset = camera.ground_projection(u, v, altitude)

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_offset = np.array([
        ground_offset[0] * cos_yaw - ground_offset[1] * sin_yaw,
        ground_offset[0] * sin_yaw + ground_offset[1] * cos_yaw,
    ])

    target_lat, target_lon = enu_to_latlon(
        world_offset[0], world_offset[1], drone_lat, drone_lon
    )
    return target_lat, target_lon


def detection_to_world_homography(
    bbox_center: Tuple[float, float],
    homography: HomographyMapper
) -> Tuple[float, float]:
    """
    Use Homography for pixel → world GPS conversion.
    Method B (recommended).

    Args:
        bbox_center: (u, v) pixel coordinate
        homography: HomographyMapper instance

    Returns: (latitude, longitude)
    """
    u, v = bbox_center
    return homography.pixel_to_gps(u, v)


# =============================================================================
# 9. Real-time 3D Map Viewer (OpenCV software projection)
# =============================================================================
class RealtimeMapViewer:
    """Fast 3D viewer using OpenCV software projection (no matplotlib overhead)."""
    def __init__(self, fence=None, window_name="3D Coordinate System - Live", window_size=800):
        self.window_name = window_name
        self.window_size = window_size
        self.fence = fence
        self.ref_lat = self.ref_lon = 0
        if fence:
            self.ref_lat, self.ref_lon = fence.center()
        self.cam_dist = 120.0
        self.cam_azim = -60.0
        self.cam_elev = 25.0
        self._map_range = 70.0
        self._mouse_down = False
        self._last_azim = self.cam_azim
        self._last_elev = self.cam_elev
        self._dt = 0.05
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, window_size, window_size)
        cv2.setMouseCallback(self.window_name, self._on_mouse)
        self._is_open = True

    def _on_mouse(self, event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._mouse_down = True
            self._mx, self._my = x, y
            self._last_azim, self._last_elev = self.cam_azim, self.cam_elev
        elif event == cv2.EVENT_MOUSEMOVE and self._mouse_down:
            self.cam_azim = self._last_azim + (x - self._mx) * 0.5
            self.cam_elev = max(-89, min(89, self._last_elev - (y - self._my) * 0.5))
        elif event == cv2.EVENT_LBUTTONUP:
            self._mouse_down = False
        elif event == cv2.EVENT_MOUSEWHEEL:
            self.cam_dist *= 0.9 if flags > 0 else 1.1
            self.cam_dist = max(20, min(500, self.cam_dist))

    def _project(self, x, y, z):
        a, e = math.radians(self.cam_azim), math.radians(self.cam_elev)
        dx, dy, dz = x, y, z - self.cam_dist
        x1 = dx * math.cos(a) - dz * math.sin(a)
        z1 = dx * math.sin(a) + dz * math.cos(a)
        y2 = dy * math.cos(e) - z1 * math.sin(e)
        z2 = dy * math.sin(e) + z1 * math.cos(e)
        if abs(z2) < 0.1:
            return (0, 0, -9999)
        f = 600
        ws = self.window_size
        return (int(f * x1 / z2 + ws // 2), int(f * y2 / z2 + ws // 2), z2)

    def _draw_axes(self, img):
        o = self._project(0, 0, 0)
        if o[2] < 0:
            return
        r = self._map_range * 0.7
        for dx, dy, dz, c, lb in [(r, 0, 0, (0, 0, 255), 'E'),
                                   (0, r, 0, (0, 255, 0), 'N'),
                                   (0, 0, r, (255, 0, 0), 'U')]:
            p = self._project(dx, dy, dz)
            if p[2] > 0:
                cv2.arrowedLine(img, (o[0], o[1]), (p[0], p[1]), c, 2, tipLength=0.08)
                cv2.putText(img, lb, (p[0] + 5, p[1] - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
        gs = self._map_range / 5
        for i in range(-5, 6):
            g = i * gs
            for p1, p2 in [((g, -self._map_range, 0), (g, self._map_range, 0)),
                           ((-self._map_range, g, 0), (self._map_range, g, 0))]:
                a, b = self._project(*p1), self._project(*p2)
                if a[2] > 0 and b[2] > 0:
                    cv2.line(img, (a[0], a[1]), (b[0], b[1]), (60, 60, 60), 1)

    def _draw_fence(self, img):
        if not self.fence:
            return
        pts = []
        for lat, lon in self.fence.corners_latlon:
            e, n = latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon)
            p = self._project(e, n, 0)
            if p[2] > 0:
                pts.append((p[0], p[1]))
        if len(pts) >= 3:
            cv2.polylines(img, [np.array(pts, dtype=np.int32).reshape(-1, 1, 2)], True, (0, 0, 255), 2)
            for px, py in pts:
                cv2.circle(img, (px, py), 4, (0, 0, 255), -1)

    def _draw_drone(self, img, e, n, alt, yaw=None):
        p = self._project(e, n, alt)
        if p[2] < 0:
            return
        cv2.circle(img, (p[0], p[1]), 8, (255, 128, 0), -1)
        cv2.circle(img, (p[0], p[1]), 8, (255, 255, 255), 1)
        if yaw is not None:
            cv2.arrowedLine(img, (p[0], p[1]),
                           (int(p[0] + 15 * math.cos(-yaw)),
                            int(p[1] + 15 * math.sin(-yaw))),
                           (255, 255, 0), 2)
        gp = self._project(e, n, 0)
        if gp[2] > 0:
            cv2.line(img, (p[0], p[1]), (gp[0], gp[1]), (100, 100, 255), 1)
            cv2.circle(img, (gp[0], gp[1]), 3, (255, 128, 0), -1)

    def _draw_objects(self, img, objects):
        """Draw detected object positions as colored markers."""
        if not objects:
            return
        for item in objects:
            e, n = float(item[0]), float(item[1])
            c = float(item[2]) if len(item) >= 3 else 0.5
            p = self._project(e, n, 0)
            if p[2] < 0:
                continue
            cv2.circle(img, (p[0], p[1]), 4, (0, int(255 * c), int(255 * (1 - c))), -1)
            cv2.circle(img, (p[0], p[1]), 4, (255, 255, 255), 1)

    def _draw_spatial_analysis(self, img, result):
        """Draw spatial analysis results (boundary, clusters, outliers)."""
        if not result or result['n_objects'] == 0:
            return

        # Boundary hull
        hull = result.get('hull', [])
        if len(hull) >= 3:
            pts = []
            for lat, lon in hull:
                e, n = latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon)
                p = self._project(e, n, 0)
                if p[2] > 0:
                    pts.append((p[0], p[1]))
            if len(pts) >= 3:
                cv2.polylines(img, [np.array(pts, dtype=np.int32).reshape(-1, 1, 2)],
                             True, (0, 165, 255), 2)

        # Weighted center
        ce, cn = result.get('center_enu', (0, 0))
        if ce != 0:
            p = self._project(ce, cn, 0)
            if p[2] > 0:
                cv2.drawMarker(img, (p[0], p[1]), (0, 255, 255), cv2.MARKER_STAR, 20, 2)

        # Boundary points
        for lat, lon in result.get('boundary_points', []):
            p = self._project(*latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon), 0)
            if p[2] > 0:
                cv2.drawMarker(img, (p[0], p[1]), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 12, 2)

        # Outliers
        for lat, lon in result.get('outliers', []):
            p = self._project(*latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon), 0)
            if p[2] > 0:
                cv2.drawMarker(img, (p[0], p[1]), (255, 0, 255), cv2.MARKER_DIAMOND, 10, 2)

    def update(self, drone_enu=None, drone_yaw=None, drone_altitude=30.0,
               object_positions=None, spatial_result=None, extra_info=None):
        """Update the 3D view. Called each frame."""
        import time
        t0 = time.time()
        s = self.window_size
        img = np.zeros((s, s, 3), dtype=np.uint8)
        img.fill(25)

        self._draw_axes(img)
        self._draw_fence(img)
        if drone_enu:
            self._draw_drone(img, drone_enu[0], drone_enu[1], drone_altitude, drone_yaw)
        if object_positions:
            self._draw_objects(img, object_positions)
        if spatial_result:
            self._draw_spatial_analysis(img, spatial_result)

        # Info panel
        lines = (extra_info or [])[:4]
        if spatial_result:
            f = spatial_result
            lines.append(f"Obj:{f['n_objects']} Cls:{f['n_clusters']} Out:{len(f['outliers'])} Bnd:{len(f.get('boundary_points', []))}")
        for i, l in enumerate(lines):
            cv2.putText(img, l, (10, 20 + i * 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        self._dt = max(0.001, time.time() - t0)
        cv2.putText(img, f"FPS:{1 / self._dt:.0f}", (s - 90, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.imshow(self.window_name, img)

    def is_open(self):
        try:
            r = cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE)
            return r >= 0
        except Exception:
            return self._is_open

    def close(self):
        try:
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass


# =============================================================================
# 10. Static Plot Function (matplotlib-based, for saving snapshots)
# =============================================================================
def plot_coordinate_map(
    drone_lat: float = None, drone_lon: float = None,
    object_positions: List[Tuple[float, float, float]] = None,
    fence: FenceConfig = None,
    spatial_analysis: Dict[str, Any] = None,
    save_path: str = None,
    title: str = 'UAV Coordinate System - Top View'
):
    """
    Plot top-down coordinate map with fence, drone, and object positions.

    Args:
        drone_lat, drone_lon: Drone GPS position
        object_positions: [(latitude, longitude, confidence), ...]
        fence: Fence configuration
        spatial_analysis: SpatialAnalyzer analysis result (optional)
        save_path: Save path for output image (optional)
        title: Chart title
    """
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
    except ImportError:
        print("[Coordinate] matplotlib required: pip install matplotlib")
        return

    # Determine reference point
    if fence:
        ref_lat, ref_lon = fence.center()
    elif object_positions and len(object_positions) > 0:
        ref_lat = sum(p[0] for p in object_positions) / len(object_positions)
        ref_lon = sum(p[1] for p in object_positions) / len(object_positions)
    elif drone_lat is not None:
        ref_lat, ref_lon = drone_lat, drone_lon
    else:
        ref_lat, ref_lon = 47.6415, -122.1402

    fig, ax = plt.subplots(figsize=(12, 10))

    # ---- Fence ----
    if fence:
        fence_enu = np.array([
            latlon_to_enu(lat, lon, ref_lat, ref_lon)
            for lat, lon in fence.corners_latlon
        ])
        polygon = Polygon(fence_enu, fill=False, edgecolor='red',
                         linewidth=2.5, label='Fence',
                         linestyle='--', alpha=0.8)
        ax.add_patch(polygon)
        ax.scatter(fence_enu[:, 0], fence_enu[:, 1],
                  c='red', marker='s', s=60, zorder=5)

    # ---- Drone ----
    if drone_lat is not None:
        de, dn = latlon_to_enu(drone_lat, drone_lon, ref_lat, ref_lon)
        ax.plot(de, dn, 'b^', markersize=18, label='Drone', zorder=10)
        circle = plt.Circle((de, dn), 30, fill=False,
                           color='blue', linestyle=':', alpha=0.4)
        ax.add_patch(circle)

    # ---- Objects ----
    if object_positions:
        obj_enu = []
        confs = []
        for p in object_positions:
            lat, lon = p[0], p[1]
            conf = p[2] if len(p) > 2 else 0.5
            e, n = latlon_to_enu(lat, lon, ref_lat, ref_lon)
            obj_enu.append([e, n])
            confs.append(conf)
        obj_enu = np.array(obj_enu)

        scatter = ax.scatter(obj_enu[:, 0], obj_enu[:, 1],
                           c=confs, cmap='YlOrRd', s=100,
                           edgecolors='black', linewidth=0.5,
                           label='Detected Objects', zorder=8,
                           vmin=0, vmax=1)
        plt.colorbar(scatter, label='Detection Confidence', ax=ax)

    # ---- Spatial Analysis Visualization ----
    if spatial_analysis:
        center_e, center_n = spatial_analysis.get('center_enu', (0, 0))
        if center_e != 0 or center_n != 0:
            ax.plot(center_e, center_n, 'r*', markersize=25,
                   markeredgecolor='white', markeredgewidth=1.5,
                   label='Center (Weighted)', zorder=12)

        hull_gps = spatial_analysis.get('hull', [])
        if len(hull_gps) >= 3:
            hull_enu = np.array([
                latlon_to_enu(lat, lon, ref_lat, ref_lon)
                for lat, lon in hull_gps
            ])
            hull_poly = Polygon(hull_enu, fill=True,
                               facecolor='orange', alpha=0.15,
                               edgecolor='darkorange', linewidth=2,
                               linestyle='-', label='Cluster Boundary')
            ax.add_patch(hull_poly)

        cluster_colors = ['green', 'purple', 'cyan', 'magenta', 'brown']
        for i, cluster in enumerate(spatial_analysis.get('clusters', [])):
            clr = cluster_colors[i % len(cluster_colors)]
            c_pts = np.array(cluster.get('positions_enu', []))
            if len(c_pts) > 0:
                ax.scatter(c_pts[:, 0], c_pts[:, 1],
                          facecolors='none', edgecolors=clr,
                          s=150, linewidths=2,
                          label=f'Cluster {i+1} ({cluster["count"]})',
                          zorder=7)

        outliers = spatial_analysis.get('outliers', [])
        if len(outliers) > 0:
            out_enu = np.array([
                latlon_to_enu(lat, lon, ref_lat, ref_lon)
                for lat, lon in outliers
            ])
            ax.scatter(out_enu[:, 0], out_enu[:, 1],
                      c='red', marker='x', s=200, linewidths=3,
                      label=f'Outliers ({len(outliers)})',
                      zorder=9)

        info_text = (
            f"Spatial Analysis:\n"
            f"  Total: {spatial_analysis['n_objects']}\n"
            f"  Clusters: {spatial_analysis['n_clusters']}\n"
            f"  Hull Area: {spatial_analysis['hull_area_m2']:.0f} m²\n"
            f"  Dispersion: {spatial_analysis['dispersion']:.2f} m\n"
            f"  Avg NN Dist: {spatial_analysis['avg_neighbor_dist']:.2f} m"
        )
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
               fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    # ---- Decoration ----
    ax.set_xlabel('East (m)', fontsize=12)
    ax.set_ylabel('North (m)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=9)
    ax.set_aspect('equal')

    # Auto-range
    all_points = []
    if fence:
        all_points.extend(fence_enu)
    if drone_lat is not None:
        all_points.append([de, dn])
    if object_positions:
        all_points.extend(obj_enu)
    if all_points:
        all_pts = np.array(all_points)
        margin = 10
        x_min, x_max = all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin
        y_min, y_max = all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[Coordinate] Saved: {save_path}")
    else:
        plt.show()
    plt.close()


# =============================================================================
# 11. Test / Demo Entry Point
# =============================================================================
if __name__ == "__main__":
    print("=" * 65)
    print("  UAV Coordinate System v2.0 - Test")
    print("=" * 65)

    # 1. Initialize camera model
    cam = BottomCamera(img_width=640, img_height=640, fov_degrees=120)
    print(f"\n[Camera] {cam.width}x{cam.height}, FOV={math.degrees(cam.fov):.1f}°")
    print(f"[Camera] fx={cam.fx:.2f}, fy={cam.fy:.2f}")

    # 2. Fence
    fence = FenceConfig()
    print(f"\n[Fence] Center: ({fence.center()[0]:.6f}, {fence.center()[1]:.6f})")

    # 3. Homography calibration
    print(f"\n[Homography] Calibrating (altitude=30m)...")
    homo = HomographyMapper(fence, cam, drone_altitude=30.0)
    print(f"[Homography] H matrix:\n{homo.get_homography_matrix()}")

    # 4. Simulated detection results (pixel coordinates)
    detections = [
        (320, 320, 0.95, 40, 40),
        (200, 200, 0.88, 35, 35),
        (450, 350, 0.85, 30, 30),
        (300, 450, 0.78, 28, 28),
        (150, 500, 0.65, 25, 25),
        (400, 250, 0.72, 32, 32),
        (280, 280, 0.90, 38, 38),
        (350, 400, 0.60, 22, 22),
    ]

    print(f"\n[Detection] {len(detections)} targets")

    # 5. Coordinate transform comparison
    drone_lat, drone_lon = 47.6415, -122.1402
    altitude = 30.0
    yaw = 0.0

    print(f"\n{'='*80}")
    print(f"{'Method':<10} {'Pixel(u,v)':<16} {'GPS Lat':<14} {'GPS Lon':<14} {'E(m)':<10} {'N(m)':<10}")
    print(f"{'='*80}")

    objects_gps = []

    for i, det in enumerate(detections):
        u, v, conf = det[0], det[1], det[2]

        # Method A: Pinhole model
        lat_a, lon_a = detection_to_world(
            (u, v), drone_lat, drone_lon, altitude, yaw, cam)
        dx_a, dy_a = gps_to_meters(drone_lat, drone_lon, lat_a, lon_a)
        print(f"{'Pinhole':<10} ({u:<3},{v:<3})       {lat_a:<14.6f} {lon_a:<14.6f} {dx_a:<10.2f} {dy_a:<10.2f}")

    print(f"{'='*80}")

    for i, det in enumerate(detections):
        u, v, conf = det[0], det[1], det[2]

        # Method B: Homography
        lat_b, lon_b = detection_to_world_homography((u, v), homo)
        dx_b, dy_b = gps_to_meters(drone_lat, drone_lon, lat_b, lon_b)
        objects_gps.append((lat_b, lon_b, conf))
        print(f"{'Homography':<10} ({u:<3},{v:<3})       {lat_b:<14.6f} {lon_b:<14.6f} {dx_b:<10.2f} {dy_b:<10.2f}")

    # 6. Spatial analysis
    print(f"\n{'='*65}")
    print("  Spatial Analysis")
    print(f"{'='*65}")

    objects_gps_full = []
    for det in detections:
        u, v, conf = det[0], det[1], det[2]
        lat, lon = detection_to_world_homography((u, v), homo)
        objects_gps_full.append((lat, lon, conf, det[3], det[4]))

    analyzer = SpatialAnalyzer(
        ref_lat=fence.center()[0],
        ref_lon=fence.center()[1])
    result = analyzer.analyze(objects_gps_full)

    print(f"\n  Weighted Center: ({result['center'][0]:.6f}, {result['center'][1]:.6f})")
    print(f"  ENU: ({result['center_enu'][0]:.2f}, {result['center_enu'][1]:.2f})")
    print(f"  Total Objects: {result['n_objects']}")
    print(f"  Clusters: {result['n_clusters']}")
    print(f"  Hull Area: {result['hull_area_m2']:.2f} m²")
    print(f"  Dispersion (std): {result['dispersion']:.2f} m")
    print(f"  Avg NN Distance: {result['avg_neighbor_dist']:.2f} m")

    if result['clusters']:
        print(f"\n  Cluster Details:")
        for i, c in enumerate(result['clusters']):
            print(f"    Cluster {i+1}: {c['count']} items, Center({c['center'][0]:.6f},{c['center'][1]:.6f}), "
                  f"Area={c['area_m2']:.1f}m²")

    # 7. Save visualization
    plot_coordinate_map(
        drone_lat, drone_lon, objects_gps, fence,
        spatial_analysis=result,
        save_path="coordinate_system_demo.png"
    )
    print(f"\n[Done] Coordinate System v2.0 test complete!")
    print(f"[Done] Visualization saved: coordinate_system_demo.png")
