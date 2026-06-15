"""
Coordinate System v2.0 — AirSim Integrated UAV Coordinate Mapping
==================================================================
Usage:
  python run_coordinate_system.py --homography        # Full mode (recommended)
  python run_coordinate_system.py --homography --only-map  # Map view only
  python run_coordinate_system.py --debug             # Debug mode (verbose errors)
  python run_coordinate_system.py --test-viewer       # Test viewer (no AirSim needed)
"""
import sys
import os
import time
import argparse
import traceback
from pathlib import Path

sys.path.insert(0, r"D:\ainsim\AirSim-1.8.1-windows\PythonClient")

import numpy as np
import math

# ============ Configuration ============
CAMERA_NAME = "bottom_center"
IMG_WIDTH, IMG_HEIGHT = 640, 640
FOV_DEG = 120
SAVE_EVERY_N = 30
# =======================================


def eprint(*args, **kwargs):
    """Print to stderr (immediate output, unbuffered)."""
    print(*args, **kwargs, file=sys.stderr, flush=True)


def test_viewer_mode():
    """Standalone test mode: no AirSim, display 3D coordinate system demo."""
    import cv2
    eprint("[Test Mode] Starting 3D coordinate system demo...")
    from coordinate_system import (FenceConfig, SpatialAnalyzer,
                                   latlon_to_enu, enu_to_latlon)
    fence = FenceConfig()
    analyzer = SpatialAnalyzer(
        ref_lat=fence.center()[0],
        ref_lon=fence.center()[1])
    ref_lat, ref_lon = fence.center()

    # Simulated object positions
    import random
    random.seed(42)
    sim_objects_base = []
    for _ in range(25):
        u = 320 + random.uniform(-80, 80)
        v = 320 + random.uniform(-80, 80)
        sim_objects_base.append((u, v, random.uniform(0.6, 0.98), 30, 30))

    eprint("[Test Mode] Running... Close window or Ctrl+C to quit")
    t0 = time.time()
    try:
        # Use RealtimeMapViewer (OpenCV version) if vispy is not available
        from coordinate_system import RealtimeMapViewer

        viewer = RealtimeMapViewer(fence, window_name="3D Coordinate System")
        while viewer.is_open():
            t = time.time() - t0
            angle = t * 0.3
            drone_e = 35 * math.cos(angle)
            drone_n = 25 * math.sin(angle)
            drone_alt = 30 + 5 * math.sin(t * 0.2)

            objects_enu = []
            objects_gps_full = []
            for j, (u, v, conf, *_) in enumerate(sim_objects_base):
                noise = math.sin(t * 0.5 + j) * 3
                se = drone_e + 3 + (u - 320) * 0.06 + noise
                sn = drone_n + 5 + (v - 320) * 0.06 + noise * 0.5
                objects_enu.append((se, sn, conf))
                slat, slon = enu_to_latlon(se, sn, ref_lat, ref_lon)
                objects_gps_full.append((slat, slon, conf, 30, 30))

            result = analyzer.analyze(objects_gps_full)

            viewer.update(
                drone_enu=(drone_e, drone_n),
                drone_yaw=angle,
                drone_altitude=drone_alt,
                object_positions=objects_enu,
                spatial_result=result,
                extra_info=[
                    f"Drone: E={drone_e:.0f} N={drone_n:.0f} H={drone_alt:.0f}",
                    f"Objects: {result['n_objects']} Clusters: {result['n_clusters']} Outliers: {len(result['outliers'])}",
                    f"Yaw: {math.degrees(angle):.0f}°",
                ],
            )

            if cv2.waitKey(50) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    except Exception as e:
        eprint(f"[Test Mode] Error: {e}")
        traceback.print_exc()
    finally:
        viewer.close()
        cv2.destroyAllWindows()
    eprint("[Test Mode] Ended")


def main():
    parser = argparse.ArgumentParser(description="Coordinate System v2.0")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--fence-only", action="store_true")
    parser.add_argument("--homography", action="store_true",
                       help="Use Homography mapping")
    parser.add_argument("--only-map", action="store_true",
                       help="Map view only (no camera window)")
    parser.add_argument("--debug", action="store_true",
                       help="Debug mode: verbose error output")
    parser.add_argument("--test-viewer", action="store_true",
                       help="Test viewer (no AirSim needed)")
    parser.add_argument("--save-dir", default="./output")
    args = parser.parse_args()

    # Standalone test mode
    if args.test_viewer:
        test_viewer_mode()
        return

    os.makedirs(args.save_dir, exist_ok=True)

    try:
        # ====== Imports (lazy, check deps first) ======
        try:
            import cv2
            from coordinate_system import (
                BottomCamera, FenceConfig, HomographyMapper,
                detection_to_world, detection_to_world_homography,
                latlon_to_enu, enu_to_latlon, gps_to_meters,
                plot_coordinate_map
            )
        except ImportError as e:
            eprint(f"[Error] Missing dependencies: {e}")
            eprint("Install: pip install opencv-python numpy scipy scikit-learn")
            sys.exit(1)

        print("=" * 55)
        print("  UAV Coordinate System v2.0")
        print("=" * 55)

        # 1. Connect AirSim
        print("\n[1/6] Connecting to AirSim...", end=' ')
        sys.stdout.flush()
        try:
            import airsim
            client = airsim.MultirotorClient()
            client.confirmConnection()
            print("OK")
        except Exception as e:
            print(f"FAILED: {e}")
            eprint("Make sure UE4 is running and Play has been clicked")
            sys.exit(1)

        # 2. Camera
        print("[2/6] Initializing camera...", end=' ')
        cam = BottomCamera(IMG_WIDTH, IMG_HEIGHT, FOV_DEG)
        print(f"{IMG_WIDTH}x{IMG_HEIGHT}, FOV={FOV_DEG}°")

        # 3. Fence
        print("[3/6] Fence...", end=' ')
        fence = FenceConfig()
        ref_lat, ref_lon = fence.center()
        print(f"Center ({ref_lat:.4f}, {ref_lon:.4f})")

        # 4. Homography
        homo = None
        if args.homography:
            print("[4/6] Homography calibration...", end=' ')
            try:
                state = client.getMultirotorState()
                altitude = abs(state.kinematics_estimated.position.z_val) or 30.0
                homo = HomographyMapper(fence, cam, altitude)
                print(f"OK (altitude={altitude:.1f}m)")
            except Exception as e:
                print(f"FAILED: {e}")
                if args.debug:
                    traceback.print_exc()
                eprint("[Warning] Homography failed, falling back to pinhole model")
                args.homography = False
        else:
            print("[4/6] Using pinhole model")

        # 5. 3D Map Viewer
        print("[5/6] 3D coordinate system viewer...", end=' ')
        try:
            from coordinate_system import RealtimeMapViewer
            viewer = RealtimeMapViewer(fence, window_size=800,
                                       window_name="3D Coordinate System - Live")
            print("OK (drag to rotate)")
        except Exception as e:
            print(f"FAILED: {e}")
            if args.debug:
                traceback.print_exc()
            sys.exit(1)

        print("\n" + "=" * 55)
        print("  Running... (Q=Quit)")
        print("=" * 55)

        # ====== Main Loop ======
        frame_count = 0
        consecutive_errors = 0

        # ---- Auto-takeoff (SimpleFlight only, PX4 uses MAVLink) ----
        print("\n[Takeoff] Requesting takeoff to 25m...", end=' ')
        sys.stdout.flush()
        try:
            client.enableApiControl(True)
            client.armDisarm(True)
            time.sleep(0.5)
            client.takeoffAsync(timeout_sec=10).join()
            try:
                client.moveToZAsync(-25.0, 1.0, timeout_sec=10).join()
            except Exception:
                pass
            print("OK")
        except Exception as e:
            print(f"\n[Takeoff] Auto-takeoff failed (use QGC for PX4): {e}")
            try:
                state = client.getMultirotorState()
                alt = abs(state.kinematics_estimated.position.z_val)
                print(f"[Takeoff] Current altitude: {alt:.1f}m")
            except Exception:
                pass
        time.sleep(2)

        while True:
            try:
                frame_count += 1

                # ---- AirSim data ----
                gps = client.getGpsData()
                drone_lat = gps.gnss.geo_point.latitude
                drone_lon = gps.gnss.geo_point.longitude
                drone_alt = gps.gnss.geo_point.altitude

                imu = client.getImuData()
                q = imu.orientation
                sin_yaw = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
                cos_yaw = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
                yaw = math.atan2(sin_yaw, cos_yaw)

                state = client.getMultirotorState()
                altitude = abs(state.kinematics_estimated.position.z_val)

                # ENU coordinates
                drone_e, drone_n = latlon_to_enu(
                    drone_lat, drone_lon, ref_lat, ref_lon)

                # ---- Camera image ----
                if not args.only_map:
                    responses = client.simGetImages([
                        airsim.ImageRequest(
                            CAMERA_NAME, airsim.ImageType.Scene, False, False)
                    ])
                    if responses and responses[0] and responses[0].image_data_uint8:
                        r = responses[0]
                        img = np.frombuffer(
                            r.image_data_uint8, dtype=np.uint8
                        ).reshape(IMG_HEIGHT, IMG_WIDTH, 3)

                # ---- Update viewer ----
                extra_info = [
                    f"Drone: ({drone_lat:.6f}, {drone_lon:.6f})",
                    f"ENU: ({drone_e:.1f}, {drone_n:.1f}) m | Alt: {altitude:.1f} m",
                    f"Yaw: {math.degrees(yaw):.1f}°",
                    f"Mode: {'Homography' if args.homography else 'Pinhole'}",
                ]

                try:
                    viewer.update(
                        drone_enu=(drone_e, drone_n),
                        drone_yaw=yaw,
                        drone_altitude=altitude,
                        extra_info=extra_info,
                    )
                except Exception as e:
                    if args.debug:
                        eprint(f"\n[Viewer Error] {e}")

                if not viewer.is_open():
                    print("\n3D window closed")
                    break

                consecutive_errors = 0

            except Exception as e:
                consecutive_errors += 1
                eprint(f"\n[Loop Error #{consecutive_errors}] {e}")
                traceback.print_exc()
                if consecutive_errors > 5:
                    eprint(f"\n[FATAL] {consecutive_errors} consecutive errors, exiting")
                    break
                time.sleep(1.0)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        eprint(f"\n[Fatal Error] {e}")
        if args.debug:
            traceback.print_exc()
    finally:
        try:
            viewer.close()
        except Exception:
            pass
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:
            pass
        print("\nExited")


if __name__ == "__main__":
    main()
