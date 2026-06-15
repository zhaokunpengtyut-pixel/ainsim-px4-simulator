"""
VisPy-based real-time 3D viewer for UAV coordinate system.
GPU-accelerated OpenGL rendering for 60fps+ performance.

Usage:
    viewer = VispyViewer(fence)
    while viewer.is_open():
        viewer.update(drone_enu, yaw, alt, objects, analysis)
        viewer.process_events()
        time.sleep(0.01)
    viewer.close()
"""
from vispy import scene, app
from vispy.scene import visuals
import numpy as np
import math


class VispyViewer:
    """GPU-accelerated 3D viewer using VisPy/OpenGL."""

    def __init__(self, fence=None, window_name="3D Coordinate System - Live",
                 window_size=800):
        from coordinate_system import latlon_to_enu

        self.fence = fence
        self._latlon_to_enu = latlon_to_enu
        if fence:
            self.ref_lat, self.ref_lon = fence.center()
        else:
            self.ref_lat = self.ref_lon = 0

        self.canvas = scene.SceneCanvas(
            title=window_name, size=(window_size, window_size),
            keys='interactive', bgcolor='#1a1a1a', show=True
        )
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = 'turntable'
        self.view.camera.center = (0, 0, 15)
        self.view.camera.distance = 120
        self.view.camera.fov = 45

        self._map_range = 70.0
        self._size = window_size
        self._artists = {}

        self._build_scene()

    def _build_scene(self):
        """Create all 3D visuals (one-time setup)."""
        # ---- 3D Axes (E=red, N=green, U=blue) ----
        r = self._map_range * 0.7
        axis_data = {
            'E': ((0, 0, 0), (r, 0, 0), (1, 0, 0, 1)),
            'N': ((0, 0, 0), (0, r, 0), (0, 1, 0, 1)),
            'U': ((0, 0, 0), (0, 0, r), (0, 0, 1, 1)),
        }
        for label, (start, end, color) in axis_data.items():
            arr = np.array([start, end], dtype=np.float32)
            line = visuals.Line(pos=arr, color=color, width=3, method='gl')
            self.view.add(line)
            self._artists[f'axis_{label}'] = line

        # ---- Grid on ground plane ----
        grid_lines = []
        gs = self._map_range / 5
        for i in range(-5, 6):
            g = i * gs
            grid_lines.append((g, -self._map_range, 0, g, self._map_range, 0))
            grid_lines.append((-self._map_range, g, 0, self._map_range, g, 0))
        grid_arr = np.array(grid_lines, dtype=np.float32).reshape(-1, 3)
        grid_colors = np.tile([0.3, 0.3, 0.3, 0.5], (len(grid_arr), 1))
        grid = visuals.Line(pos=grid_arr, color=grid_colors, width=1,
                           method='gl', connect='segments')
        self.view.add(grid)
        self._artists['grid'] = grid

        # ---- Fence (red polygon) ----
        if self.fence:
            fence_pts = []
            for lat, lon in self.fence.corners_latlon:
                e, n = self._latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon)
                fence_pts.append([e, n, 0])
            if fence_pts:
                fence_pts.append(fence_pts[0])
                fence_arr = np.array(fence_pts, dtype=np.float32)
                fence = visuals.Line(pos=fence_arr, color=(1, 0, 0, 0.9),
                                    width=2, method='gl')
                self.view.add(fence)
                self._artists['fence'] = fence

        # ---- Markers for objects ----
        objects = visuals.Markers()
        self.view.add(objects)
        self._artists['objects'] = objects

        # ---- Markers for drone ----
        drone = visuals.Markers()
        self.view.add(drone)
        self._artists['drone'] = drone

        # ---- Hull boundary ----
        hull = visuals.Line(color=(1, 0.65, 0, 0.8), width=2, method='gl')
        self.view.add(hull)
        self._artists['hull'] = hull

        # ---- Boundary points markers ----
        boundary = visuals.Markers()
        self.view.add(boundary)
        self._artists['boundary'] = boundary

        # ---- Outlier markers ----
        outlier = visuals.Markers()
        self.view.add(outlier)
        self._artists['outlier'] = outlier

        # ---- Center marker ----
        center = visuals.Markers()
        self.view.add(center)
        self._artists['center'] = center

        # ---- Info text ----
        self._info_texts = []
        for i in range(6):
            t = visuals.Text('', pos=(10, 20 + i * 20), font_size=10,
                             color=(0.8, 0.8, 0.8, 1), anchor_x='left')
            self.view.add(t)
            self._info_texts.append(t)
        self._artists['info'] = self._info_texts

    # --- Public API ---

    def update(self, drone_enu=None, drone_yaw=None, drone_altitude=30.0,
               object_positions=None, spatial_result=None, extra_info=None):
        """Update all 3D objects. Called each frame."""
        from coordinate_system import latlon_to_enu

        # ---- Drone ----
        if drone_enu:
            e, n = drone_enu[0], drone_enu[1]
            self._artists['drone'].set_data(
                np.array([[e, n, drone_altitude]], dtype=np.float32),
                face_color=(1, 0.5, 0, 1), size=16,
                edge_color=(1, 1, 1, 1), edge_width=1
            )
        else:
            self._artists['drone'].set_data(np.empty((0, 3), dtype=np.float32))

        # ---- Objects ----
        if object_positions and len(object_positions) > 0:
            o_pos = []
            o_col = []
            for item in object_positions:
                e, n = float(item[0]), float(item[1])
                c = float(item[2]) if len(item) >= 3 else 0.5
                o_pos.append([e, n, 0])
                o_col.append([0, c, 1 - c, 0.9])
            self._artists['objects'].set_data(
                np.array(o_pos, dtype=np.float32),
                face_color=np.array(o_col, dtype=np.float32),
                size=8, edge_color=(1, 1, 1, 0.5), edge_width=1
            )
        else:
            self._artists['objects'].set_data(np.empty((0, 3), dtype=np.float32))

        # ---- Spatial analysis ----
        if spatial_result and spatial_result['n_objects'] > 0:
            # Hull boundary
            hull = spatial_result.get('hull', [])
            if len(hull) >= 3:
                hull_pts = []
                for lat, lon in hull:
                    e, n = latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon)
                    hull_pts.append([e, n, 0])
                hull_pts.append(hull_pts[0])
                self._artists['hull'].set_data(
                    np.array(hull_pts, dtype=np.float32)
                )
            else:
                self._artists['hull'].set_data(np.empty((0, 3), dtype=np.float32))

            # Boundary points
            boundary_data = spatial_result.get('boundary_points', [])
            if boundary_data:
                b_pos = []
                for lat, lon in boundary_data:
                    en, nn = latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon)
                    b_pos.append([en, nn, 0])
                self._artists['boundary'].set_data(
                    np.array(b_pos, dtype=np.float32),
                    face_color=(1, 0, 0, 1), size=14,
                    symbol='x', edge_color=(1, 0, 0, 1), edge_width=2
                )
            else:
                self._artists['boundary'].set_data(np.empty((0, 3), dtype=np.float32))

            # Outliers
            outlier_data = spatial_result.get('outliers', [])
            if outlier_data:
                o_pos = []
                for lat, lon in outlier_data:
                    en, nn = latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon)
                    o_pos.append([en, nn, 0])
                self._artists['outlier'].set_data(
                    np.array(o_pos, dtype=np.float32),
                    face_color=(1, 0, 1, 1), size=12,
                    symbol='diamond', edge_color=(1, 0, 1, 1), edge_width=1
                )
            else:
                self._artists['outlier'].set_data(np.empty((0, 3), dtype=np.float32))

            # Center
            ce, cn = spatial_result.get('center_enu', (0, 0))
            if ce != 0:
                self._artists['center'].set_data(
                    np.array([[ce, cn, 0]], dtype=np.float32),
                    face_color=(1, 1, 0, 1), size=20,
                    symbol='star', edge_color=(1, 1, 1, 1), edge_width=1
                )
            else:
                self._artists['center'].set_data(np.empty((0, 3), dtype=np.float32))
        else:
            for k in ['hull', 'boundary', 'outlier', 'center']:
                self._artists[k].set_data(np.empty((0, 3), dtype=np.float32))

        # ---- Info panel ----
        info_lines = (extra_info or [])[:4]
        if spatial_result:
            f = spatial_result
            info_lines.append(
                f"Obj:{f['n_objects']} Cls:{f['n_clusters']} "
                f"Out:{len(f['outliers'])} Bnd:{len(f.get('boundary_points', []))}"
            )
        for i, t in enumerate(self._info_texts):
            if i < len(info_lines):
                t.text = info_lines[i]
                t.pos = (10, self._size - 20 - i * 22)
                t.visible = True
            else:
                t.visible = False

        self.canvas.update()

    def is_open(self):
        """Check if window is still open."""
        try:
            return self.canvas.native.isVisible()
        except Exception:
            return False

    def process_events(self):
        """Call periodically from main loop to process GUI events."""
        app.process_events()

    def close(self):
        """Close the viewer."""
        try:
            self.canvas.close()
        except Exception:
            pass
