#!/usr/bin/python3
"""
sma3_main_combined.py
─────────────────────────────────────────────────────────────────────────────
RealSense + ArUco + SAM3 detection, talking to the Doosan over TCP.

Per pick, SMA3 now sends:
    pick:X,Y,Z,yaw,gripper        ← dynamic, from the camera
    drop_pose:<pose_name>         ← NAMED pose from poses_V2.json
    run:<doosan_sequence>         ← typically "pick_and_place"

Each step in sequence_robotic_arm.json specifies:
    "part":            part key from PARTS
    "drop_pose":       name of the Doosan pose to drop into
    "doosan_sequence": (optional) sequence name to invoke
    "count":           how many picks of this part to do
    "retries":         detection retries per pick
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from collections import deque
import json
import logging
import os
import threading
import time

import torch
from scipy.ndimage import binary_erosion, distance_transform_edt
from PIL import Image, ImageDraw
from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor

from comm import Link


# ============================================================
# 0)  LOGGING
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - [%(levelname)s] - %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("sma3")

latest_aruco_mask = None 
# ============================================================
# 0a) DOOSAN TCP CONFIG
# ============================================================
USE_DOOSAN              = True
DOOSAN_HOST             = "192.168.108.43"
DOOSAN_PORT             = 9000
DOOSAN_CONNECT_TIMEOUT  = 30
DOOSAN_READY_TIMEOUT    = 60
DOOSAN_SEQUENCE_TIMEOUT = 600

# Doosan sequence to invoke if a step does not specify "doosan_sequence".
DEFAULT_DOOSAN_SEQUENCE = "pick_and_place"


# ============================================================
# 0b) ZYZ <-> R <-> quat
# ============================================================
def zyz_to_R(rx, ry, rz):
    a, b, c = np.radians([rx, ry, rz])
    Rz1 = np.array([[np.cos(a), -np.sin(a), 0],
                    [np.sin(a),  np.cos(a), 0],
                    [0,          0,         1]])
    Ry_ = np.array([[ np.cos(b), 0, np.sin(b)],
                    [ 0,         1, 0        ],
                    [-np.sin(b), 0, np.cos(b)]])
    Rz2 = np.array([[np.cos(c), -np.sin(c), 0],
                    [np.sin(c),  np.cos(c), 0],
                    [0,          0,         1]])
    return Rz1 @ Ry_ @ Rz2


def R_to_zyz(R):
    if abs(R[2, 2]) < 1 - 1e-6:
        ry = np.arctan2(np.sqrt(R[0, 2] ** 2 + R[1, 2] ** 2), R[2, 2])
        rx = np.arctan2(R[1, 2], R[0, 2])
        rz = np.arctan2(R[2, 1], -R[2, 0])
    else:
        ry = 0.0 if R[2, 2] > 0 else np.pi
        rx = 0.0
        rz = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([rx, ry, rz])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
        if i == 0:
            s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1 - R[0, 0] + R[1, 1] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s;                z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1 - R[0, 0] - R[1, 1] + R[2, 2]) * 2
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return np.array([w, x, y, z])


def quat_to_R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def slerp(q1, q2, t):
    d = np.dot(q1, q2)
    if d < 0: q2, d = -q2, -d
    if d > 0.9995:
        return (q1 + t * (q2 - q1)) / np.linalg.norm(q1 + t * (q2 - q1))
    th = np.arccos(d)
    return (np.sin((1 - t) * th) * q1 + np.sin(t * th) * q2) / np.sin(th)


# ============================================================
# 0c) Grid-pose generator from 4 taught corners
# ============================================================
def generate_grid_poses(corners, NX, NY, verbose=True):
    pos   = [np.array(p[:3], dtype=float) for p in corners]
    quats = [R_to_quat(zyz_to_R(*p[3:])) for p in corners]
    poses = []
    for j in range(NY):
        v = j / (NY - 1) if NY > 1 else 0.0
        for i in range(NX):
            u = i / (NX - 1) if NX > 1 else 0.0
            P = ((1 - u) * (1 - v) * pos[0] + u * (1 - v) * pos[1]
                 + (1 - u) * v       * pos[2] + u * v       * pos[3])
            qt = slerp(quats[0], quats[1], u)
            qb = slerp(quats[2], quats[3], u)
            q  = slerp(qt, qb, v)
            rx, ry, rz = R_to_zyz(quat_to_R(q))
            poses.append([float(P[0]), float(P[1]), float(P[2]),
                          float(rx),    float(ry),    float(rz)])
            if verbose:
                print(f"  [{i},{j}] posx({P[0]:.2f}, {P[1]:.2f}, {P[2]:.2f}, "
                      f"{rx:.2f}, {ry:.2f}, {rz:.2f})")
    return poses


# ============================================================
# 1)  MARKER LAYOUT IN THE ROBOT FRAME
# ============================================================
MARKER_SIZE = 0.037
HALF        = MARKER_SIZE / 2

MARKER_CENTRES = {
    1: np.array([0.71566, -0.04276, 0.0500]),
    0: np.array([0.45748, -0.04659, 0.0500]),
    5: np.array([0.70805, 0.1228, 0.0500]),
    2: np.array([0.45497, 0.12492, 0.0500]),
}
MARKER_ORIENTATIONS_ZYZ_DEG = {
    1: (174.90, 180.0, 174.90),
    0: (170.99, 180.0, 170.99),
    5: (  8.49, 180.0,   8.49),
    2: ( 10.80, 180.0,  10.00),
}
CORNER_OFFSETS_FROM_CENTRE_IMG_ORDER = np.array([
    [+HALF, -HALF, 0.0],
    [-HALF, -HALF, 0.0],
    [-HALF, +HALF, 0.0],
    [+HALF, +HALF, 0.0],
])

WORLD_MARKERS     = dict(MARKER_CENTRES)
WORLD_CORNERS_IMG = {mid: WORLD_MARKERS[mid][None, :]
                          + CORNER_OFFSETS_FROM_CENTRE_IMG_ORDER
                     for mid in WORLD_MARKERS}
EXPECTED_IDS = set(WORLD_MARKERS.keys())

_xs = [c[0] for c in MARKER_CENTRES.values()]
_ys = [c[1] for c in MARKER_CENTRES.values()]
WORKSPACE_X_MIN, WORKSPACE_X_MAX = min(_xs) - MARKER_SIZE, max(_xs) + MARKER_SIZE
WORKSPACE_Y_MIN, WORKSPACE_Y_MAX = min(_ys) - MARKER_SIZE, max(_ys) + MARKER_SIZE

_ID_TL_IMG = 1
_ID_TR_IMG = 0
_ID_BL_IMG = 5
_ID_BR_IMG = 2


def _sanity_check_layout():
    print("\n── LAYOUT SANITY CHECK ──")
    pairs = [
        (_ID_TL_IMG, _ID_TR_IMG, "top row    (1 -> 0)"),
        (_ID_BL_IMG, _ID_BR_IMG, "bottom row (5 -> 2)"),
        (_ID_TL_IMG, _ID_BL_IMG, "left col   (1 -> 5)"),
        (_ID_TR_IMG, _ID_BR_IMG, "right col  (0 -> 2)"),
    ]
    for a, b, label in pairs:
        d = WORLD_MARKERS[b] - WORLD_MARKERS[a]
        dist = float(np.linalg.norm(d[:2]))
        print(f"  {label}: dX={d[0]*1000:+7.1f}  dY={d[1]*1000:+7.1f}  "
              f"dZ={d[2]*1000:+6.1f}  |XY|={dist*1000:6.1f} mm")
    zs = [c[2] for c in WORLD_MARKERS.values()]
    print(f"  Z spread: {(max(zs)-min(zs))*1000:.1f} mm")
    print("──────────────────────────\n")


def _make_grid_corner(mid):
    c_m  = MARKER_CENTRES[mid]
    rx, ry, rz = MARKER_ORIENTATIONS_ZYZ_DEG[mid]
    return [float(c_m[0] * 1000.0),
            float(c_m[1] * 1000.0),
            float(c_m[2] * 1000.0),
            float(rx), float(ry), float(rz)]


GRID_CORNERS = [
    _make_grid_corner(_ID_TL_IMG),
    _make_grid_corner(_ID_TR_IMG),
    _make_grid_corner(_ID_BL_IMG),
    _make_grid_corner(_ID_BR_IMG),
]
GRID_NX, GRID_NY = 5, 4
GRID_POSES_FILE  = "grid_poses.json"
RUN_GRID_ON_M    = False


# ============================================================
# 2) CALIB / WORKSPACE / UI
# ============================================================
COLOR_W, COLOR_H = 1280, 720
DEPTH_W, DEPTH_H = 1280, 720
FPS              = 30
UI_SCALE         = COLOR_W / 640.0

AVG_WINDOW       = 30
MAX_REPROJ_PX    = 3.0 * UI_SCALE
CALIB_FILE       = "world_calib.json"

CAPTURE_PATH       = "captured.png"
DETECT_OUTPUT_PATH = "output_detect.jpg"

COORD_UNITS = "mm"
SHOW_GRID   = False
USE_CORNERS = False


# ============================================================
# 3) SEQUENCE / PIPELINE SETTINGS
# ============================================================
SEQUENCE_FILE      = "sequence_robotic_arm.json"
POST_PICK_SETTLE_S = 0.8
FRAME_FLUSH_COUNT  = 5


# ============================================================
# 4) GRIPPERS
# ============================================================
GRIPPERS = {
    "parallel": {
        "description": "Two-finger parallel gripper",
        "max_opening_mm":        80,
        "min_opening_mm":        1,
        "finger_length_mm":      1001,
        "finger_width_mm":       5,
        "finger_thickness_mm":   5,
        "max_payload_g":         500,
        "grasp_margin_mm":       4,
        "side_clearance_mm":     8,
        "approach_clearance_mm": 15,
        "finger_pad_px":         20,
        "px_per_mm":             None,
    },
    "suction": {
        "description": "Suction cup",
        "cup_diameter_mm":       8,
        "max_payload_g":         300,
        "flatness_tolerance_mm": 1.5,
        "edge_margin_mm":        5,
        "min_object_area_px":    600,
        "px_per_mm":             None,
    },
}


# ============================================================
# 5) PARTS
# ============================================================
PARTS = {
    "black_block": {
        "name": "Black block", "prompt": "black object", "gripper": "suction",
        "tcp": "default",
        "selection": {"min_score": 0.60, "min_area_px": 500,
                      "max_aspect_ratio": 2.5, "min_compactness": 0.50,
                      "require_in_workspace": True},
    },
    "vilt": {
        "name": "vilt sticker", "prompt": "black object", "gripper": "suction",
        "tcp": "snoeks1",
        "rotate_for_angle": True,
        "selection": {"min_score": 0.75, "min_area_px": 4000,
                      "require_in_workspace": True},
    },
    "manuel": {
        "name": "manuel", "prompt": "text", "gripper": "suction",
        "tcp": "default",
        "selection": {"min_score": 0.45, "min_area_px": 1000,
                      "require_in_workspace": False},
    },
    "blue object": {
        "name": "blue object", "prompt": "blue object", "gripper": "parallel",
        "tcp": "default",
        "selection": {"min_score": 0.45, "min_area_px": 300,
                      "require_in_workspace": True},
    },
    "black nail": {
        "name": "black nail", "prompt": "black object", "gripper": "parallel",
        "tcp": "default",
        "selection": {"min_score": 0.45, "min_area_px": 300, "max_area_px": 500,
                      "require_in_workspace": True},
    },
    "plastic bags": {
        "name": "plastic bags", "prompt": "plastic ", "gripper": "parallel",
        "tcp": "default",
        "selection": {"min_score": 0.4, "min_area_px": 300,
                      "require_in_workspace": True},
    },
    "black cap big": {
        "name": "black cap big", "prompt": "black object", "gripper": "suction",
        "tcp": "default",
        "selection": {"min_score": 0.75, "min_area_px": 300, "max_area_px": 1500,
                      "require_in_workspace": True},
    },
    "Black bumper": {
        "name": "Black bumper", "prompt": "black object", "gripper": "suction",
        "tcp": "default",
        "selection": {"min_score": 0.75, "require_in_workspace": True},
    },
}
ACTIVE_PART = "black_block"
PART_KEYS   = list(PARTS.keys())

# ============================================================
# 6) REALSENSE
# ============================================================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, FPS)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale  = depth_sensor.get_depth_scale()

spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 2)
spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
spatial.set_option(rs.option.filter_smooth_delta, 20)
temporal = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
temporal.set_option(rs.option.filter_smooth_delta, 20)
hole_filling = rs.hole_filling_filter(1)

intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
camera_matrix = np.array([[intr.fx, 0, intr.ppx],
                          [0, intr.fy, intr.ppy],
                          [0,       0,        1]], dtype=np.float64)
dist_coeffs = np.array(intr.coeffs, dtype=np.float64)
print(f"Stream: {intr.width}x{intr.height}  fx={intr.fx:.1f} fy={intr.fy:.1f}")


# ============================================================
# 7) ARUCO
# ============================================================
aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_params.cornerRefinementMethod        = cv2.aruco.CORNER_REFINE_SUBPIX
aruco_params.cornerRefinementWinSize       = 5
aruco_params.cornerRefinementMaxIterations = 50
aruco_params.cornerRefinementMinAccuracy   = 0.01
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


# ============================================================
# 8) GLOBALS
# ============================================================
H_pix2robot        = None
H_pix2robot_locked = None
H_robot2pix        = None
H_robot2pix_locked = None
calibration_locked = False

latest_depth_image = None
latest_color_image = None

marker_corner_history = {mid: deque(maxlen=AVG_WINDOW)
                         for mid in WORLD_MARKERS}
clicked_points        = []
last_reproj_err_px    = None
current_px_per_mm     = None


# ============================================================
# 9) SAM3 LOAD
# ============================================================
print("\nLoading SAM3 model (one-time)…")
_t0 = time.time()
_sam3_model     = build_sam3_image_model().float()
_sam3_processor = Sam3Processor(_sam3_model)
print(f"SAM3 ready in {time.time()-_t0:.1f}s\n")


# ============================================================
# 10) HELPERS
# ============================================================
def unit_scale():
    return {"mm": 1000.0, "cm": 100.0, "m": 1.0}[COORD_UNITS]


def fmt_xyz(xyz_m, compact=False):
    if xyz_m is None:
        return "X=---  Y=---  Z=---"
    s = unit_scale()
    x, y, z = xyz_m[0]*s, xyz_m[1]*s, xyz_m[2]*s
    if compact:
        return f"X={x:7.1f} Y={y:7.1f} Z={z:7.1f} {COORD_UNITS}"
    return f"X = {x:8.2f}  Y = {y:8.2f}  Z = {z:8.2f}  {COORD_UNITS}"


def apply_H(H, pts_uv):
    pts  = np.asarray(pts_uv, dtype=np.float64).reshape(-1, 2)
    ones = np.ones((pts.shape[0], 1))
    hom  = np.hstack([pts, ones]) @ H.T
    hom /= hom[:, 2:3]
    return hom[:, :2]


def interp_plane_Z(x, y):
    z_tl = WORLD_MARKERS[_ID_TL_IMG][2]
    z_tr = WORLD_MARKERS[_ID_TR_IMG][2]
    z_bl = WORLD_MARKERS[_ID_BL_IMG][2]
    z_br = WORLD_MARKERS[_ID_BR_IMG][2]
    x_l = WORLD_MARKERS[_ID_TL_IMG][0]
    x_r = WORLD_MARKERS[_ID_TR_IMG][0]
    y_t = WORLD_MARKERS[_ID_TL_IMG][1]
    y_b = WORLD_MARKERS[_ID_BL_IMG][1]
    fx = (x - x_l) / (x_r - x_l) if abs(x_r - x_l) > 1e-9 else 0.0
    fy = (y - y_t) / (y_b - y_t) if abs(y_b - y_t) > 1e-9 else 0.0
    fx = float(np.clip(fx, -0.5, 1.5))
    fy = float(np.clip(fy, -0.5, 1.5))
    z_top    = (1 - fx) * z_tl + fx * z_tr
    z_bottom = (1 - fx) * z_bl + fx * z_br
    return (1 - fy) * z_top + fy * z_bottom


def get_active_H():
    if calibration_locked:
        return H_pix2robot_locked, H_robot2pix_locked
    return H_pix2robot, H_robot2pix


def pixel_to_world(u, v, depth_image=None):
    H, _ = get_active_H()
    if H is None:
        return None
    xy = apply_H(H, [[u, v]])[0]
    x, y = float(xy[0]), float(xy[1])
    z = interp_plane_Z(x, y)
    return np.array([x, y, z])


def pixel_yaw_to_robot_yaw(u, v, angle_deg_pixel):
    H, _ = get_active_H()
    if H is None:
        return angle_deg_pixel
    rad = np.radians(angle_deg_pixel)
    eps = 10.0
    p0 = apply_H(H, [[u, v]])[0]
    p1 = apply_H(H, [[u + eps * np.cos(rad),
                      v + eps * np.sin(rad)]])[0]
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    if abs(dx) < 1e-12 and abs(dy) < 1e-12:
        return angle_deg_pixel
    return float(np.degrees(np.arctan2(dy, dx)))


def in_workspace(world_xyz):
    x, y = world_xyz[0], world_xyz[1]
    return (WORKSPACE_X_MIN - 0.01 <= x <= WORKSPACE_X_MAX + 0.01) and \
           (WORKSPACE_Y_MIN - 0.01 <= y <= WORKSPACE_Y_MAX + 0.01)


def get_depth_median(depth_image, u, v, size=7):
    if depth_image is None:
        return 0.0
    h, w = depth_image.shape
    hp = size // 2
    y1, y2 = max(0, int(v) - hp), min(h, int(v) + hp + 1)
    x1, x2 = max(0, int(u) - hp), min(w, int(u) + hp + 1)
    patch = depth_image[y1:y2, x1:x2]
    valid = patch[patch > 0]
    if len(valid) < 5:
        return 0.0
    return float(np.median(valid)) * depth_scale


# ============================================================
# 11) HOMOGRAPHY CALIBRATION
# ============================================================
def fit_homography(pix_pts, robot_xy):
    pix_pts  = np.asarray(pix_pts,  dtype=np.float64)
    robot_xy = np.asarray(robot_xy, dtype=np.float64)
    if len(pix_pts) < 4:
        return None, None, None, None
    if len(pix_pts) == 4:
        H = cv2.getPerspectiveTransform(pix_pts.astype(np.float32),
                                        robot_xy.astype(np.float32))
        H = H.astype(np.float64)
    else:
        H, _ = cv2.findHomography(pix_pts, robot_xy, cv2.RANSAC, 0.001)
    if H is None:
        return None, None, None, None
    H_inv    = np.linalg.inv(H)
    proj_pix = apply_H(H_inv, robot_xy)
    err      = np.linalg.norm(proj_pix - pix_pts, axis=1)
    return H, H_inv, float(err.mean()), err


def reorder_corners_to_image_frame(detected_corners, marker_centre_pix,
                                   H_inv_seed, mid):
    expected_world = WORLD_CORNERS_IMG[mid][:, :2]
    expected_pix   = apply_H(H_inv_seed, expected_world)
    used, order = set(), []
    for i in range(4):
        d = np.linalg.norm(detected_corners - expected_pix[i], axis=1)
        for j in np.argsort(d):
            if j not in used:
                order.append(int(j))
                used.add(int(j))
                break
    return detected_corners[order]


def save_calibration(H, err_px):
    data = {
        "timestamp":            time.strftime("%Y-%m-%d %H:%M:%S"),
        "method":               "planar_homography",
        "use_corners":          USE_CORNERS,
        "H_pix2robot":          H.tolist(),
        "marker_size_m":        MARKER_SIZE,
        "marker_centres_robot": {str(k): v.tolist()
                                 for k, v in MARKER_CENTRES.items()},
        "mean_reproj_err_px":   err_px,
        "intrinsics": {"fx": intr.fx, "fy": intr.fy,
                       "ppx": intr.ppx, "ppy": intr.ppy,
                       "width": intr.width, "height": intr.height},
    }
    with open(CALIB_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Calibration saved -> {CALIB_FILE}")


def load_calibration():
    if not os.path.exists(CALIB_FILE):
        print(f"No calibration file: {CALIB_FILE}")
        return None, None
    with open(CALIB_FILE, "r") as f:
        data = json.load(f)
    if "H_pix2robot" not in data:
        print(f"  {CALIB_FILE} does not contain an H_pix2robot homography.")
        return None, None
    H = np.array(data["H_pix2robot"], dtype=np.float64)
    H_inv = np.linalg.inv(H)
    print(f"Calibration loaded "
          f"(reproj={data.get('mean_reproj_err_px','?')} px)")
    return H, H_inv


def update_px_per_mm_from_aruco(corners_list, ids_array):
    global current_px_per_mm
    if ids_array is None or len(ids_array) == 0:
        return
    sides_px = []
    for c in corners_list:
        pts = c.reshape(4, 2)
        for i in range(4):
            sides_px.append(np.linalg.norm(pts[i] - pts[(i + 1) % 4]))
    if not sides_px:
        return
    avg_side_px = float(np.mean(sides_px))
    px_per_mm   = avg_side_px / (MARKER_SIZE * 1000.0)
    current_px_per_mm = px_per_mm
    for g in GRIPPERS.values():
        g["px_per_mm"] = px_per_mm


# ============================================================
# 12) DETECTION HELPERS
# ============================================================
def mask_to_bool(mask):
    m = mask.cpu().numpy()
    if m.ndim == 3:
        m = m.squeeze(0)
    return m.astype(bool)


def principal_axes(bool_mask):
    ys, xs = np.where(bool_mask)
    if len(xs) < 5:
        return 0.0, np.array([1., 0.]), np.array([0., 1.]), 0.0, 0.0
    cx_m, cy_m = xs.mean(), ys.mean()
    pts = np.column_stack([xs - cx_m, ys - cy_m]).astype(float)
    cov = pts.T @ pts / len(pts)
    vals, vecs = np.linalg.eigh(cov)
    major = vecs[:, 1]
    minor = np.array([-major[1], major[0]])
    proj_major = pts @ major
    proj_minor = pts @ minor
    hw_major = (proj_major.max() - proj_major.min()) / 2.0
    hw_minor = (proj_minor.max() - proj_minor.min()) / 2.0
    angle = float(np.degrees(np.arctan2(major[1], major[0])))
    return angle, major, minor, hw_major, hw_minor


def compactness_score(bool_mask):
    eroded = binary_erosion(bool_mask, iterations=1)
    boundary = bool_mask ^ eroded
    perim = boundary.sum()
    area = bool_mask.sum()
    return float(4 * np.pi * area / (perim ** 2)) if perim else 0.0


def bbox_aspect_ratio_xyxy(box):
    x0, y0, x1, y1 = [float(v) for v in box.tolist()]
    w = max(x1 - x0, 1.0); h = max(y1 - y0, 1.0)
    r = w / h
    return max(r, 1.0 / r)


def mask_centroid(bool_mask):
    ys, xs = np.where(bool_mask)
    if len(xs) == 0:
        return None
    return float(xs.mean()), float(ys.mean())


# ============================================================
# 13) GRIPPER FEASIBILITY
# ============================================================
def _stamp_rect(canvas, centre, axis_along, axis_perp,
                half_along, half_perp):
    H, W = canvas.shape
    corners = np.array([
        centre - half_along * axis_along - half_perp * axis_perp,
        centre + half_along * axis_along - half_perp * axis_perp,
        centre + half_along * axis_along + half_perp * axis_perp,
        centre - half_along * axis_along + half_perp * axis_perp,
    ], dtype=np.int32)
    cv2.fillConvexPoly(canvas.view(np.uint8).reshape(H, W), corners, 1)
    return canvas


def gripper_can_grasp(part, bool_mask, occupied_others):
    g = GRIPPERS[part["gripper"]]
    ppm = g.get("px_per_mm")
    info = {"gripper": part["gripper"]}

    if ppm is None or ppm <= 0:
        return False, "no px_per_mm (no ArUco visible)", info

    if part["gripper"] == "parallel":
        ang, major, minor, hw_major, hw_minor = principal_axes(bool_mask)
        c = mask_centroid(bool_mask)
        if c is None:
            return False, "empty mask", info
        centre = np.array(c)
        info.update({"centre": centre, "major": major, "minor": minor,
                     "hw_major": hw_major, "hw_minor": hw_minor,
                     "angle_deg": ang})

        width_mm  = (2.0 * hw_minor) / ppm
        length_mm = (2.0 * hw_major) / ppm
        info["width_mm"]  = width_mm
        info["length_mm"] = length_mm

        if width_mm < g["min_opening_mm"]:
            return False, f"too thin ({width_mm:.1f}mm < {g['min_opening_mm']}mm)", info
        usable_max = g["max_opening_mm"] - g["grasp_margin_mm"]
        if width_mm > usable_max:
            return False, f"too wide ({width_mm:.1f}mm > {usable_max:.1f}mm)", info
        if length_mm > g["finger_length_mm"]:
            return False, f"too long ({length_mm:.1f}mm > {g['finger_length_mm']}mm)", info

        finger_w_px  = (g["finger_width_mm"]     * ppm) / 2.0
        finger_t_px  = (g["finger_thickness_mm"] * ppm) / 2.0
        side_clr_px  = g["side_clearance_mm"]    * ppm
        grasp_mar_px = g["grasp_margin_mm"]      * ppm

        reach = hw_minor + grasp_mar_px + finger_t_px
        p1 = centre + reach * minor
        p2 = centre - reach * minor
        info["finger_pts"] = (p1, p2)
        info["finger_half_along"] = finger_w_px + side_clr_px
        info["finger_half_perp"]  = finger_t_px + side_clr_px

        H, W = bool_mask.shape
        foot = np.zeros((H, W), dtype=bool)
        for fp in (p1, p2):
            _stamp_rect(foot, fp, major, minor,
                        finger_w_px + side_clr_px,
                        finger_t_px + side_clr_px)

        if np.any(foot & occupied_others):
            return False, "neighbour in finger path", info

        return True, "ok", info

    elif part["gripper"] == "suction":
        c = mask_centroid(bool_mask)
        if c is None:
            return False, "empty mask", info
        cx, cy = c
        info["centre"] = np.array([cx, cy])
        info["angle_deg"] = 0.0

        area_px = int(bool_mask.sum())
        if area_px < g["min_object_area_px"]:
            return False, f"area too small ({area_px} < {g['min_object_area_px']})", info

        dt = distance_transform_edt(bool_mask)
        max_r_px = float(dt.max())
        cup_r_px = (g["cup_diameter_mm"] / 2.0) * ppm
        info["cup_r_px"] = cup_r_px
        info["edge_margin_px"] = g["edge_margin_mm"] * ppm
        if max_r_px < cup_r_px:
            return False, (f"no flat spot for cup "
                           f"(max_r={max_r_px:.0f}px < cup_r={cup_r_px:.0f}px)"), info

        H, W = bool_mask.shape
        yy, xx = np.ogrid[:H, :W]
        clr_r = cup_r_px + g["edge_margin_mm"] * ppm
        disc = (xx - cx) ** 2 + (yy - cy) ** 2 <= clr_r ** 2
        if np.any(disc & occupied_others):
            return False, "neighbour within edge margin", info

        return True, "ok", info

    return False, f"unknown gripper {part['gripper']!r}", info

_current_tcp_on_robot = None     # tracked client-side

def ensure_tcp(tcp_name: str, timeout: float = 10.0) -> bool:
    """
    Tell the Doosan to activate `tcp_name` if it isn't already.

    Returns True on success, False on timeout / unknown TCP.
    """
    global _current_tcp_on_robot
    if not tcp_name:
        tcp_name = "default"
    if tcp_name == _current_tcp_on_robot:
        return True
    if ROBOT.name != "doosan":
        # DummyRobot: just print and remember
        print(f"   [DUMMY] set_tcp:{tcp_name}")
        _current_tcp_on_robot = tcp_name
        return True

    # Reuse the same wait-for-done plumbing as run_named_sequence,
    # but watch for an `ack:set_tcp:<name>` line.
    with ROBOT._lock:
        ROBOT.last_done  = None
        ROBOT.last_error = None
    ROBOT._done_event.clear()

    # We need a tiny extension to DoosanRobot to surface ack:set_tcp.
    # Easiest is to set a flag from _on_message — see patch below.
    ROBOT._pending_tcp_ack = tcp_name
    ROBOT._tcp_ack_event = threading.Event()
    ROBOT._send(f"set_tcp:{tcp_name}")

    if not ROBOT._tcp_ack_event.wait(timeout=timeout):
        logger.error(f"  TCP switch to '{tcp_name}' timed out.")
        return False
    if ROBOT.last_error:
        logger.error(f"  TCP switch error: {ROBOT.last_error}")
        return False
    _current_tcp_on_robot = tcp_name
    logger.info(f"  ✔ TCP now: {tcp_name}")
    return True
# ============================================================
# 14) SELECTION FILTER
# ============================================================
def passes_selection(part, score, area, aspect, compact, world_xyz):
    sel = part["selection"]
    if "min_score" in sel and score < sel["min_score"]:
        return False, f"score {score:.2f} < {sel['min_score']}"
    if "min_area_px" in sel and area < sel["min_area_px"]:
        return False, f"area {area} < {sel['min_area_px']}"
    if "max_area_px" in sel and area > sel["max_area_px"]:
        return False, f"area {area} > {sel['max_area_px']}"
    if "max_aspect_ratio" in sel and aspect > sel["max_aspect_ratio"]:
        return False, f"aspect {aspect:.2f} > {sel['max_aspect_ratio']}"
    if "min_compactness" in sel and compact < sel["min_compactness"]:
        return False, f"compact {compact:.2f} < {sel['min_compactness']}"
    if sel.get("require_in_workspace", False):
        if world_xyz is None:
            return False, "no world XYZ"
        if not in_workspace(world_xyz):
            return False, "outside workspace"
    return True, "ok"


# ============================================================
# 15) DRAW
# ============================================================
PALETTE = [(80,140,255,90),(80,200,120,90),(255,200,50,90),
           (200,80,255,90),(50,220,220,90),(255,80,80,90)]


def draw_results(image, candidates, best_idx):
    overlay = Image.new("RGBA", image.size, (0,0,0,0))
    for i, cand in enumerate(candidates):
        layer = np.zeros((*cand["mask_bool"].shape, 4), dtype=np.uint8)
        col = PALETTE[i % len(PALETTE)] if cand["accepted"] else (255,80,80,70)
        layer[cand["mask_bool"]] = col
        overlay = Image.alpha_composite(overlay, Image.fromarray(layer, "RGBA"))
    image = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")
    draw = ImageDraw.Draw(image)

    for i, cand in enumerate(candidates):
        x0, y0, x1, y1 = [int(v) for v in cand["box"].tolist()]
        is_best = (i == best_idx)
        if is_best:
            col = "lime"; w = 4
        elif cand["accepted"]:
            col = "#90ee90"; w = 2
        else:
            col = "#ff5050"; w = 1
        draw.rectangle([x0, y0, x1, y1], outline=col, width=w)

        ginfo = cand.get("ginfo", {})
        if ginfo.get("gripper") == "parallel" and "finger_pts" in ginfo:
            p1, p2 = ginfo["finger_pts"]
            centre = ginfo["centre"]
            major  = ginfo["major"]
            minor  = ginfo["minor"]
            ha = ginfo["finger_half_along"]
            hp = ginfo["finger_half_perp"]
            L = ginfo["hw_minor"] + hp + 4
            a = centre + L * minor
            b = centre - L * minor
            ax_col = "cyan" if cand["accepted"] else "#ff9090"
            draw.line([a[0], a[1], b[0], b[1]], fill=ax_col, width=2)
            fc = "lime" if is_best else ("orange" if cand["accepted"] else "red")
            for fp in (p1, p2):
                corners = [
                    fp - ha * major - hp * minor,
                    fp + ha * major - hp * minor,
                    fp + ha * major + hp * minor,
                    fp - ha * major + hp * minor,
                ]
                pts = [(float(c[0]), float(c[1])) for c in corners]
                draw.polygon(pts, outline=fc)
                draw.ellipse([fp[0]-3, fp[1]-3, fp[0]+3, fp[1]+3], fill=fc)

        elif ginfo.get("gripper") == "suction" and "centre" in ginfo:
            cx, cy = ginfo["centre"]
            cup_r = ginfo.get("cup_r_px", 8)
            clr_r = cup_r + ginfo.get("edge_margin_px", 0)
            fc = "lime" if is_best else ("orange" if cand["accepted"] else "red")
            draw.ellipse([cx-cup_r, cy-cup_r, cx+cup_r, cy+cup_r],
                         outline=fc, width=2)
            for a in range(0, 360, 20):
                draw.arc([cx-clr_r, cy-clr_r, cx+clr_r, cy+clr_r],
                         start=a, end=a+10, fill="cyan", width=1)

        label = f"#{i+1} s={cand['score']:.2f}"
        if not cand["accepted"]:
            label += f"  DROP: {cand['reason']}"
        draw.text((x0, max(0, y0 - 14)), label, fill=col)

    if best_idx is not None:
        cand = candidates[best_idx]
        cx, cy = cand["pick_px"]
        r = 22
        draw.ellipse([cx-r, cy-r, cx+r, cy+r], outline="red", width=3)
        draw.line([cx-2*r, cy, cx+2*r, cy], fill="red", width=2)
        draw.line([cx, cy-2*r, cx, cy+2*r], fill="red", width=2)
        draw.text((cx + r + 4, cy - 8), "PICK", fill="red")
    return image


# ============================================================
# 16) CORE DETECTOR
# ============================================================
def detect_part(part_key, bgr_frame, depth_image, save_annot=True, show=True):
    part = PARTS[part_key]
    g    = GRIPPERS[part["gripper"]]
    print("\n" + "═" * 78)
    print(f" SAM3 DETECT — part: {part['name']!r}  prompt: {part['prompt']!r}")
    print(f" gripper: {part['gripper']}  px_per_mm: "
          f"{g.get('px_per_mm') if g.get('px_per_mm') else 'N/A'}")
    print("═" * 78)

    cv2.imwrite(CAPTURE_PATH, bgr_frame)
    image = Image.open(CAPTURE_PATH).convert("RGB")

    t0 = time.time()
    try:
        with torch.autocast(device_type="cuda", dtype=torch.float32):
            state  = _sam3_processor.set_image(image)
            output = _sam3_processor.set_text_prompt(state=state,
                                                     prompt=part["prompt"])
    except Exception as e:
        print(f"  SAM3 failed: {e}")
        return {"best": None, "candidates": [], "annotated_path": None}

    masks, boxes, scores = output["masks"], output["boxes"], output["scores"]
    if len(scores) == 0:
        print("  No detections at all.")
        return {"best": None, "candidates": [], "annotated_path": None}
    print(f"  SAM3 returned {len(scores)} raw detections in {time.time()-t0:.1f}s")

    bool_masks = [mask_to_bool(m) for m in masks]
    candidates = []
    s_units = unit_scale()
    print(f"\n  {'#':>2}  {'score':>5}  {'area':>6}  {'asp':>4}  {'cmp':>4}  "
          f"{'X':>7} {'Y':>7} {'Z':>7} ({COORD_UNITS})   verdict")
    print("  " + "-" * 80)

    for i in range(len(scores)):
        bm      = bool_masks[i]
        if latest_aruco_mask is not None:
            overlap = np.any(bm & (latest_aruco_mask > 0))
            if overlap:
                print(f"  {i+1:>2}  BLOCKED (overlaps ArUco marker)")
                candidates.append({
                    "mask_bool": bm,
                    "box": boxes[i],
                    "score": float(scores[i].item()),
                    "area": int(bm.sum()),
                    "aspect": 0,
                    "compact": 0,
                    "world": None,
                    "pick_px": (0, 0),
                    "accepted": False,
                    "reason": "overlaps ArUco marker",
                    "ginfo": {"gripper": part["gripper"]},
                })
                continue
        score   = float(scores[i].item())
        area    = int(bm.sum())
        aspect  = bbox_aspect_ratio_xyxy(boxes[i])
        compact = compactness_score(bm)
        cen     = mask_centroid(bm)

        if latest_aruco_mask is not None:
            overlap = np.any(bm & (latest_aruco_mask > 0))
            if overlap:
                print(f"  {i+1:>2}  BLOCKED (overlaps ArUco marker)")
                candidates.append({
                    "mask_bool": bm,
                    "box": boxes[i],
                    "score": float(scores[i].item()),
                    "area": int(bm.sum()),
                    "aspect": 0,
                    "compact": 0,
                    "world": None,
                    "pick_px": (0, 0),
                    "accepted": False,
                    "reason": "overlaps ArUco marker",
                    "ginfo": {"gripper": part["gripper"]},
                })
                continue
                
        if cen is None:
            continue
        cx, cy = cen
        world  = pixel_to_world(cx, cy, depth_image)

        ok_sel, reason_sel = passes_selection(part, score, area, aspect,
                                              compact, world)

        occupied_others = np.zeros_like(bm)
        for j, bm_j in enumerate(bool_masks):
            if j != i:
                occupied_others |= bm_j

        if ok_sel:
            ok_grip, reason_grip, ginfo = gripper_can_grasp(
                part, bm, occupied_others)
        else:
            ok_grip, reason_grip, ginfo = False, "—", {"gripper": part["gripper"]}

        accepted = ok_sel and ok_grip
        reason   = reason_sel if not ok_sel else (
                       reason_grip if not ok_grip else "ok")

        if world is None:
            world_str = "   ---     ---     ---  "
        else:
            world_str = (f"{world[0]*s_units:7.2f} "
                         f"{world[1]*s_units:7.2f} "
                         f"{world[2]*s_units:7.2f}")
        verdict = "OK" if accepted else f"DROP ({reason})"
        print(f"  {i+1:>2}  {score:>5.2f}  {area:>6}  "
              f"{aspect:>4.2f}  {compact:>4.2f}  "
              f"{world_str}   {verdict}")

        candidates.append({
            "mask_bool": bm, "box": boxes[i],
            "score":   score, "area":   area,
            "aspect":  aspect,"compact": compact,
            "world":   world, "pick_px": (int(round(cx)), int(round(cy))),
            "accepted": accepted, "reason": reason,
            "ginfo":   ginfo,
        })

    accepted_idx = [k for k, c in enumerate(candidates) if c["accepted"]]
    best_idx, best_payload = None, None
    if not accepted_idx:
        print("\n  No candidate passed all filters.")
    else:
        accepted_idx.sort(key=lambda k: candidates[k]["score"], reverse=True)
        best_idx = accepted_idx[0]
        c = candidates[best_idx]

        ang_pix = float(c["ginfo"].get("angle_deg", 0.0))
        if part["gripper"] == "parallel":
            pix_jaw_angle = ang_pix + 90.0
        elif part["gripper"] == "suction" and part.get("rotate_for_angle"):
            # Square sticker: align suction tip rotation with the object's edge
            pix_jaw_angle = ang_pix
        else:
            pix_jaw_angle = 0.0
        yaw_deg = pixel_yaw_to_robot_yaw(c["pick_px"][0], c["pick_px"][1],
                                        pix_jaw_angle)
        # Normalise to (-90, 90] for a square (180° symmetry)
        if part["gripper"] == "suction" and part.get("rotate_for_angle"):
            yaw_deg = ((yaw_deg + 90.0) % 180.0) - 90.0
        else:
            yaw_deg = ((yaw_deg + 180.0) % 360.0) - 180.0

        print("\n  ── BEST PICK ──")
        print(f"   instance #     : {best_idx+1}")
        print(f"   score          : {c['score']:.3f}")
        print(f"   pixel          : {c['pick_px']}")
        print(f"   yaw (robot)    : {yaw_deg:+.1f}°")
        if c["world"] is None:
            print("   ROBOT XYZ      : (no calibration)")
            world_xyz = None
        else:
            world_xyz = c["world"]
            print(f"   {fmt_xyz(world_xyz)}")

        best_payload = {
            "pixel":   c["pick_px"],
            "world":   world_xyz,
            "yaw_deg": yaw_deg,
            "score":   c["score"],
            "part":    part_key,
            "gripper": part["gripper"],
        }

    annotated_path = None
    if save_annot:
        annotated = draw_results(image.copy(), candidates, best_idx)
        annotated.save(DETECT_OUTPUT_PATH)
        annotated_path = DETECT_OUTPUT_PATH
        print(f"\n   annotated img: {DETECT_OUTPUT_PATH}")
        if show:
            ann_bgr = cv2.cvtColor(np.array(annotated), cv2.COLOR_RGB2BGR)
            cv2.namedWindow("SAM3 Detections", cv2.WINDOW_NORMAL)
            cv2.imshow("SAM3 Detections", ann_bgr)
            cv2.waitKey(1)
    print("═" * 78 + "\n")

    return {"best": best_payload, "candidates": candidates,
            "annotated_path": annotated_path}


def run_sam3_detect(bgr_frame, depth_image):
    detect_part(ACTIVE_PART, bgr_frame, depth_image,
                save_annot=True, show=True)


# ============================================================
# 17a) DUMMY ROBOT
# ============================================================
class DummyRobot:
    name = "dummy"

    def __init__(self):
        self.sequences = []
        self.connected = True

    def connect(self, **kw): return True
    def wait_ready(self, **kw): return True
    def stop(self): pass

    def send_pick(self, x, y, z, yaw, gripper):
        print(f"   [DUMMY] pick:{x:.2f},{y:.2f},{z:.2f},{yaw:.2f},{gripper}")

    def send_drop_pose(self, pose_name):
        print(f"   [DUMMY] drop_pose:{pose_name}")

    def home(self):
        print("   [DUMMY] home")

    def run_named_sequence(self, name, timeout=600):
        print(f"   [DUMMY] run:{name}  (simulated)")
        time.sleep(0.5)
        print(f"   [DUMMY] done:{name}")
        return True


# ============================================================
# 17b) DOOSAN ROBOT (TCP via comm.Link)
# ============================================================
class DoosanRobot:
    name = "doosan"

    def __init__(self, host, port):
        self.host  = host
        self.port  = port
        self.link  = None
        self.connected = False

        self.sequences   = []
        self.ready       = False
        self.last_done   = None
        self.last_error  = None
        self.last_started = None

        self._lock        = threading.Lock()
        self._done_event  = threading.Event()

    # ---------- internal ------------------------------------------------
    def _on_message(self, msg: str) -> None:
        logger.info(f"[Doosan] {msg}")

        # Handle TCP ack OUTSIDE the lock — it has its own event
        if msg.startswith("ack:set_tcp:"):
            parts = msg.split(":", 2)
            if len(parts) == 3:
                got = parts[2].strip()
                pending = getattr(self, "_pending_tcp_ack", None)
                evt = getattr(self, "_tcp_ack_event", None)
                if pending == got and evt is not None:
                    evt.set()
            return

        with self._lock:
            if msg == "doosan_ready":
                self.ready = True
            elif msg.startswith("sequences:"):
                seqs = msg.split(":", 1)[1].split(",")
                self.sequences = [s.strip() for s in seqs if s.strip()]
                logger.info(f"Doosan sequences: {self.sequences}")
            elif msg.startswith("started:"):
                self.last_started = msg.split(":", 1)[1]
            elif msg.startswith("done:"):
                self.last_done = msg.split(":", 1)[1]
                self._done_event.set()
            elif msg.startswith("error:"):
                self.last_error = msg
                self._done_event.set()


    def _send(self, cmd: str) -> None:
        if not self.connected or self.link is None:
            logger.error(f"[Doosan] not connected — dropping cmd: {cmd}")
            return
        logger.info(f"  →  {cmd}")
        self.link.send(cmd)

    # ---------- lifecycle -----------------------------------------------
    def connect(self, timeout: float = DOOSAN_CONNECT_TIMEOUT) -> bool:
        self.link = Link(role="client", host=self.host, port=self.port,
                         on_message=self._on_message)
        self.link.start()
        logger.info(f"Connecting to Doosan at {self.host}:{self.port} ...")
        if not self.link.wait_until_connected(timeout=timeout):
            logger.error("Doosan connect timeout.")
            self.link.stop(); self.link = None
            return False
        self.connected = True
        logger.info("Doosan socket connected.")
        return True

    def wait_ready(self, timeout: float = DOOSAN_READY_TIMEOUT) -> bool:
        start = time.time()
        while time.time() - start < timeout:
            if self.ready:
                time.sleep(0.5)   # let sequences: line arrive too
                return True
            time.sleep(0.2)
        logger.error("Doosan never sent 'doosan_ready'.")
        return False

    def stop(self) -> None:
        try:
            if self.link is not None:
                self.link.stop()
        finally:
            self.connected = False
            self.link = None

    # ---------- high-level pattern --------------------------------------
    def send_pick(self, x, y, z, yaw, gripper):
        self._send(f"pick:{x:.3f},{y:.3f},{z:.3f},{yaw:.3f},{gripper}")

    def send_drop_pose(self, pose_name: str):
        self._send(f"drop_pose:{pose_name}")

    def home(self):
        self._send("home")

    def run_named_sequence(self, name, timeout=DOOSAN_SEQUENCE_TIMEOUT) -> bool:
        with self._lock:
            self.last_done  = None
            self.last_error = None
        self._done_event.clear()
        self._send(f"run:{name}")

        start = time.time()
        while time.time() - start < timeout:
            if self._done_event.wait(timeout=1.0):
                self._done_event.clear()
                with self._lock:
                    if self.last_done == name:
                        logger.info(f"  ✔ done:{name}")
                        return True
                    if self.last_done is not None:
                        logger.warning(
                            f"  got done:{self.last_done} (expected {name})")
                        return False
                    if self.last_error is not None:
                        logger.error(f"  Doosan error: {self.last_error}")
                        return False
        logger.error(f"  Timed out waiting for done:{name}")
        return False


# ============================================================
# 17c) Global ROBOT handle
# ============================================================
ROBOT = DummyRobot()


# ============================================================
# 17d) GRID RUNNER
# ============================================================
def run_grid_poses(corners=None, NX=None, NY=None,
                   save_path=None, drive_robot=None):
    corners     = corners     if corners     is not None else GRID_CORNERS
    NX          = NX          if NX          is not None else GRID_NX
    NY          = NY          if NY          is not None else GRID_NY
    save_path   = save_path   if save_path   is not None else GRID_POSES_FILE
    drive_robot = drive_robot if drive_robot is not None else RUN_GRID_ON_M

    print("\n" + "█" * 78)
    print(f" GRID-POSE GENERATOR  ({NX}×{NY} = {NX*NY} poses)")
    for name, p in zip(["P00(TL,id1)", "P10(TR,id0)",
                        "P01(BL,id5)", "P11(BR,id2)"], corners):
        print(f"   {name}: posx({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}, "
              f"{p[3]:.2f}, {p[4]:.2f}, {p[5]:.2f})")
    print("█" * 78)

    poses = generate_grid_poses(corners, NX, NY, verbose=True)
    payload = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "NX": NX, "NY": NY,
        "corners": corners, "poses": poses,
        "format": "X_mm, Y_mm, Z_mm, Rx_deg, Ry_deg, Rz_deg  (Doosan ZYZ)",
    }
    with open(save_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\n  Saved -> {save_path}")
    print("█" * 78 + "\n")
    return poses


# ============================================================
# 18) WORLD ↔ ROBOT
# ============================================================
def world_to_robot(world_xyz_m, world_yaw_deg=0.0):
    p_robot = np.asarray(world_xyz_m, dtype=np.float64) * 1000.0
    return p_robot, world_yaw_deg


# ============================================================
# 19) CAMERA + OPERATOR HELPERS
# ============================================================
def grab_fresh_frame(flush=FRAME_FLUSH_COUNT):
    global latest_color_image, latest_depth_image, latest_aruco_mask

    last_color = last_depth = None
    for _ in range(max(1, flush)):
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        cf = aligned.get_color_frame()
        df = aligned.get_depth_frame()
        if not cf or not df:
            continue

        df = spatial.process(df)
        df = temporal.process(df)
        df = hole_filling.process(df)

        last_depth = np.asanyarray(df.as_depth_frame().get_data())
        last_color = np.asanyarray(cf.get_data())

    if last_color is not None:
        latest_color_image = last_color
        latest_depth_image = last_depth

        corners, ids, _ = detector.detectMarkers(last_color)
        update_px_per_mm_from_aruco(corners, ids)

        # ---- BUILD ARUCO EXCLUSION MASK ----
        h, w = last_color.shape[:2]
        aruco_mask = np.zeros((h, w), dtype=np.uint8)

        if ids is not None:
            for c in corners:
                pts = c.reshape(4, 2).astype(np.int32)
                cv2.fillConvexPoly(aruco_mask, pts, 255)

        # Slightly dilate so we also block near-border grasps
        aruco_mask = cv2.dilate(aruco_mask, np.ones((25, 25), np.uint8))

        latest_aruco_mask = aruco_mask

    return last_color, last_depth



def operator_prompt(msg, allowed=("placed",)):
    extras = ("skip", "abort")
    while True:
        try:
            ans = input(f"   >>> {msg}: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return "abort"
        if ans in allowed or ans in extras:
            return ans
        print(f"       (type one of: {', '.join(list(allowed)+list(extras))})")


# ============================================================
# 20) SEQUENCE RUNNER  (Doosan-aware, named drop pose)
# ============================================================
def run_sequence(path=SEQUENCE_FILE):
    if not os.path.exists(path):
        print(f"[ERR] sequence file not found: {path}")
        return
    with open(path, "r") as f:
        items = json.load(f)
    if not isinstance(items, list) or not items:
        print(f"[ERR] sequence file empty or wrong format: {path}")
        return

    print("\n" + "█" * 78)
    print(f" RUNNING SEQUENCE: {path}  ({len(items)} item(s))")
    print(f" Robot backend:    {ROBOT.name}")
    if getattr(ROBOT, "sequences", None):
        print(f" Doosan sequences: {ROBOT.sequences}")
    print("█" * 78)

    for step_idx, item in enumerate(items, 1):
        tray       = item.get("tray", f"#{step_idx}")
        part_key   = item.get("part")
        drop_pose  = item.get("drop_pose")
        retries    = int(item.get("retries", 3))
        count      = int(item.get("count",   1))
        doosan_seq = item.get("doosan_sequence", DEFAULT_DOOSAN_SEQUENCE)

        print("\n" + "─" * 78)
        print(f" STEP {step_idx}/{len(items)}  tray={tray}  part={part_key}  "
              f"drop_pose={drop_pose}  count={count}  retries={retries}  "
              f"doosan_seq={doosan_seq}")
        print("─" * 78)

        if part_key not in PARTS:
            print(f"  [ERR] unknown part '{part_key}' — skipping.")
            continue
        if not drop_pose:
            print(f"  [ERR] step has no 'drop_pose' — skipping.")
            continue

        if (ROBOT.name == "doosan" and ROBOT.sequences
                and doosan_seq not in ROBOT.sequences):
            print(f"   ⚠ Doosan does not list '{doosan_seq}' — "
                  f"sending anyway.")

        print(f"   put tray {tray} on the plate")
        ans = operator_prompt("type 'placed' when ready (or 'skip'/'abort')")
        if ans == "abort":
            print("   ABORT — leaving sequence.")
            return
        if ans == "skip":
            print(f"   skipping tray {tray}.")
            continue

        picked = 0
        for pick_idx in range(1, count + 1):
            print(f"\n   ── PICK {pick_idx}/{count}  "
                  f"(tray {tray}, part {part_key}) ──")

            # 1) DETECT
            best = None
            for attempt in range(1, retries + 1):
                color, depth = grab_fresh_frame()
                if color is None:
                    print("   [ERR] no camera frame — aborting step.")
                    break

                result = detect_part(part_key, color, depth,
                                     save_annot=True, show=True)
                best = result["best"]
                if best is not None and best["world"] is not None:
                    break

                remaining = retries - attempt
                print(f"\n   ⚠️  Could not find a usable '{part_key}'.")
                if remaining <= 0:
                    print("      No retries left.")
                    break
                print( "      Please SHAKE the plate so parts reposition.")
                print(f"      ({remaining} retry/retries left)")
                ans = operator_prompt(
                    "type 'shaken' when done (or 'skip'/'abort')",
                    allowed=("shaken",))
                if ans == "abort":
                    print("   ABORT — leaving sequence.")
                    return
                if ans == "skip":
                    print("   skipping this pick.")
                    best = None
                    break

            if best is None or best["world"] is None:
                print(f"   ✗ giving up on pick {pick_idx}/{count} "
                      f"for tray {tray}.")
                break

            # 2) m → mm in robot frame
            p_robot, robot_yaw = world_to_robot(best["world"], best["yaw_deg"])
            xr, yr, zr = p_robot

            print(f"   BEST XYZ (robot mm): "
                  f"X={xr:7.2f}  Y={yr:7.2f}  Z={zr:7.2f}  "
                  f"yaw={robot_yaw:+.1f}°  gripper={best['gripper']}")
            desired_tcp = PARTS[part_key].get("tcp", "default")
            if not ensure_tcp(desired_tcp):
                print(f"   ✗ Could not switch TCP to {desired_tcp}; aborting step.")
                break
            # 3) Send to Doosan
            ROBOT.send_pick(xr, yr, zr, robot_yaw, best["gripper"])
            ROBOT.send_drop_pose(drop_pose)

            ok = ROBOT.run_named_sequence(doosan_seq,
                                          timeout=DOOSAN_SEQUENCE_TIMEOUT)
            if not ok:
                print(f"   ✗ Doosan did not finish '{doosan_seq}' cleanly.")
                break

            picked += 1
            print(f"   ✓ pick {pick_idx}/{count} done "
                  f"(total picked from tray {tray}: {picked})")

            if pick_idx < count:
                time.sleep(POST_PICK_SETTLE_S)

        print(f"\n   Tray {tray} summary: {picked}/{count} picked.")

    try:
        ROBOT.home()
    except Exception:
        pass

    print("\n" + "█" * 78)
    print(" SEQUENCE COMPLETE")
    print("█" * 78 + "\n")


# ============================================================
# 21) MOUSE
# ============================================================
def mouse_callback(event, x, y, flags, param):
    global clicked_points
    if event == cv2.EVENT_LBUTTONDOWN:
        H, _ = get_active_H()
        if H is None:
            print("No calibration yet.")
            return
        xyz = pixel_to_world(x, y, latest_depth_image)
        if xyz is None:
            return
        ws = "" if in_workspace(xyz) else "  [outside workspace]"
        print(f"\n>>> Pixel ({x:4d},{y:4d})  ROBOT  {fmt_xyz(xyz)}{ws}")
        s = unit_scale()
        print(f"    move to: x={xyz[0]*s:.2f} y={xyz[1]*s:.2f} "
              f"z={xyz[2]*s:.2f} ({COORD_UNITS})")
        clicked_points.append(((x, y), xyz))
        if len(clicked_points) > 10:
            clicked_points.pop(0)
    elif event == cv2.EVENT_RBUTTONDOWN:
        clicked_points = []


cv2.namedWindow("World Calibration", cv2.WINDOW_NORMAL)
cv2.resizeWindow("World Calibration", COLOR_W, COLOR_H)
cv2.setMouseCallback("World Calibration", mouse_callback)


# ============================================================
# 22) STARTUP
# ============================================================
S_BIG, S_MED, S_SMALL = 0.55 * UI_SCALE, 0.50 * UI_SCALE, 0.42 * UI_SCALE
T_BOLD = max(1, int(round(2 * UI_SCALE * 0.7)))
T_THIN = max(1, int(round(UI_SCALE * 0.7)))
LINE_H, LINE_HS = int(round(22 * UI_SCALE)), int(round(15 * UI_SCALE))

_sanity_check_layout()

print("Marker CENTRES (in robot frame):")
for mid, c in MARKER_CENTRES.items():
    rx, ry, rz = MARKER_ORIENTATIONS_ZYZ_DEG[mid]
    print(f"  id {mid}: {fmt_xyz(c)}   Rx={rx:7.2f} Ry={ry:6.2f} Rz={rz:7.2f}")

print(f"\nActive part: {PARTS[ACTIVE_PART]['name']}")
print("Keys: L-click | SPACE=lock | C=clear | S=save | D=load | V=verify")
print("      G=grid | U=units | T=mode | X=SAM3 detect | 1..9=part")
print("      P=run sequence | M=grid-poses | ESC=quit")
for i, k in enumerate(PART_KEYS[:9]):
    print(f"    {i+1}: {PARTS[k]['name']} "
          f"(prompt={PARTS[k]['prompt']!r}, gripper={PARTS[k]['gripper']})")

_H_loaded, _H_inv_loaded = load_calibration()
if _H_loaded is not None:
    H_pix2robot_locked = _H_loaded
    H_robot2pix_locked = _H_inv_loaded
    calibration_locked = True

# Bring up Doosan link
if USE_DOOSAN:
    print("\n" + "▒" * 78)
    print(f"  Connecting to Doosan at {DOOSAN_HOST}:{DOOSAN_PORT} ...")
    print("▒" * 78)
    doosan = DoosanRobot(DOOSAN_HOST, DOOSAN_PORT)
    if doosan.connect(timeout=DOOSAN_CONNECT_TIMEOUT):
        if doosan.wait_ready(timeout=DOOSAN_READY_TIMEOUT):
            ROBOT = doosan
            print(f"  Doosan link READY. Backend = {ROBOT.name}")
            if doosan.sequences:
                print(f"  Doosan sequences: {doosan.sequences}")
        else:
            print("  Doosan never sent 'doosan_ready' — using DummyRobot.")
            doosan.stop()
    else:
        print("  Could not connect — using DummyRobot.")
else:
    print("\n  USE_DOOSAN=False — using DummyRobot.")


# ============================================================
# 23) MAIN LOOP
# ============================================================
try:
    while True:
        frames  = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        df = spatial.process(depth_frame)
        df = temporal.process(df)
        df = hole_filling.process(df)

        latest_depth_image = np.asanyarray(df.as_depth_frame().get_data())
        latest_color_image = np.asanyarray(color_frame.get_data())
        img = latest_color_image.copy()

        corners, ids, _ = detector.detectMarkers(img)
        update_px_per_mm_from_aruco(corners, ids)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            for mid_arr, m_corner in zip(ids.flatten(), corners):
                mid = int(mid_arr)
                if mid in WORLD_MARKERS:
                    pts = m_corner.reshape(4, 2).astype(np.float64)
                    marker_corner_history[mid].append(pts)

        seed_pix, seed_robot, used_ids = [], [], []
        for mid in WORLD_MARKERS:
            if len(marker_corner_history[mid]) >= 3:
                avg = np.median(np.stack(marker_corner_history[mid]), axis=0)
                centre_pix = avg.mean(axis=0)
                seed_pix.append(centre_pix)
                seed_robot.append(WORLD_MARKERS[mid][:2])
                used_ids.append(mid)

        live_H = live_H_inv = mean_err = per_err = None
        if len(used_ids) >= 4:
            H_seed, Hi_seed, e_seed, pe_seed = fit_homography(
                seed_pix, seed_robot)
            if not USE_CORNERS:
                live_H, live_H_inv = H_seed, Hi_seed
                mean_err, per_err  = e_seed, pe_seed
            else:
                pix_pts, robot_pts = [], []
                for mid in used_ids:
                    avg = np.median(np.stack(marker_corner_history[mid]),
                                    axis=0)
                    avg_img = reorder_corners_to_image_frame(
                        avg, avg.mean(axis=0), Hi_seed, mid)
                    for k in range(4):
                        pix_pts.append(avg_img[k])
                        robot_pts.append(WORLD_CORNERS_IMG[mid][k, :2])
                live_H, live_H_inv, mean_err, per_err = fit_homography(
                    pix_pts, robot_pts)

            if live_H is not None:
                last_reproj_err_px = mean_err
                if not calibration_locked:
                    H_pix2robot = live_H
                    H_robot2pix = live_H_inv

        H_active, H_active_inv = get_active_H()
        if H_active_inv is not None:
            ws_robot = np.array([
                [WORKSPACE_X_MIN, WORKSPACE_Y_MIN],
                [WORKSPACE_X_MAX, WORKSPACE_Y_MIN],
                [WORKSPACE_X_MAX, WORKSPACE_Y_MAX],
                [WORKSPACE_X_MIN, WORKSPACE_Y_MAX],
            ], dtype=np.float64)
            ws_img = apply_H(H_active_inv, ws_robot)
            cv2.polylines(img, [ws_img.astype(np.int32)],
                          isClosed=True, color=(255, 0, 255),
                          thickness=T_THIN, lineType=cv2.LINE_AA)

            for mid, c in WORLD_MARKERS.items():
                pc = apply_H(H_active_inv, [c[:2]])[0]
                u, v = int(round(pc[0])), int(round(pc[1]))
                cv2.drawMarker(img, (u, v), (0, 255, 255),
                               cv2.MARKER_CROSS, 14, T_BOLD)
                cv2.putText(img, f"id{mid}", (u + 8, v - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, S_SMALL,
                            (0, 255, 255), T_THIN, cv2.LINE_AA)

            if SHOW_GRID:
                step = 0.02
                xs = np.arange(WORKSPACE_X_MIN, WORKSPACE_X_MAX + 1e-6, step)
                ys = np.arange(WORKSPACE_Y_MIN, WORKSPACE_Y_MAX + 1e-6, step)
                for x in xs:
                    seg = apply_H(H_active_inv,
                                  [[x, WORKSPACE_Y_MIN], [x, WORKSPACE_Y_MAX]])
                    cv2.line(img, tuple(seg[0].astype(int)),
                             tuple(seg[1].astype(int)), (80, 200, 80), 1)
                for y in ys:
                    seg = apply_H(H_active_inv,
                                  [[WORKSPACE_X_MIN, y], [WORKSPACE_X_MAX, y]])
                    cv2.line(img, tuple(seg[0].astype(int)),
                             tuple(seg[1].astype(int)), (80, 200, 80), 1)

        for (px, py), xyz in clicked_points:
            in_ws = in_workspace(xyz)
            col_dot = (0, 0, 255) if in_ws else (0, 165, 255)
            cv2.circle(img, (px, py), int(6 * UI_SCALE), col_dot, -1)
            cv2.putText(img, fmt_xyz(xyz, compact=True),
                        (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, S_MED,
                        col_dot, T_THIN, cv2.LINE_AA)

        y0 = int(28 * UI_SCALE)
        p_name = PARTS[ACTIVE_PART]["name"]
        p_grip = PARTS[ACTIVE_PART]["gripper"]
        mode_str = "CORNERS(16pt)" if USE_CORNERS else "CENTRES(4pt)"
        backend_str = f"Robot={ROBOT.name}"
        if ROBOT.name == "doosan" and getattr(ROBOT, "ready", False):
            backend_str += " READY"
        cv2.putText(img,
            f"Part: {p_name}  [grip={p_grip}]  |  "
            f"Homography {mode_str}  units={COORD_UNITS}  "
            f"markers: {len(used_ids)}/4  |  {backend_str}",
            (10, y0), cv2.FONT_HERSHEY_SIMPLEX, S_BIG,
            (0, 255, 255), T_BOLD, cv2.LINE_AA)
        y0 += LINE_H

        if current_px_per_mm is not None:
            cv2.putText(img, f"scale: {current_px_per_mm:.2f} px/mm",
                        (10, y0), cv2.FONT_HERSHEY_SIMPLEX, S_SMALL,
                        (200, 200, 200), T_THIN, cv2.LINE_AA)
            y0 += LINE_HS

        if last_reproj_err_px is not None:
            col = (0, 255, 0) if last_reproj_err_px < MAX_REPROJ_PX else (0, 165, 255)
            lock_label = "[LOCKED]" if calibration_locked else "[live]"
            cv2.putText(img,
                f"reproj err: {last_reproj_err_px:5.2f} px  {lock_label}",
                (10, y0), cv2.FONT_HERSHEY_SIMPLEX, S_MED,
                (255, 0, 255) if calibration_locked else col,
                T_THIN, cv2.LINE_AA)
            y0 += LINE_HS

        cv2.putText(img,
            "L=click  SPACE=lock  S=save  D=load  V=verify  G=grid  "
            "U=units  T=mode  X=detect  1..9=part  P=seq  M=grid-poses  ESC=quit",
            (10, COLOR_H - int(12 * UI_SCALE)),
            cv2.FONT_HERSHEY_SIMPLEX, S_SMALL,
            (200, 200, 200), T_THIN, cv2.LINE_AA)

        cv2.imshow("World Calibration", img)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            break

        elif key == 32:
            if calibration_locked:
                calibration_locked = False
                H_pix2robot_locked = None
                H_robot2pix_locked = None
                print("UNLOCKED")
            elif H_pix2robot is not None:
                H_pix2robot_locked = H_pix2robot.copy()
                H_robot2pix_locked = H_robot2pix.copy()
                calibration_locked = True
                print(f"LOCKED (reproj={last_reproj_err_px:.2f} px)")
            else:
                print("Cannot lock — no homography yet.")

        elif key in (ord('c'), ord('C')):
            for mid in marker_corner_history:
                marker_corner_history[mid].clear()
            print("Marker history cleared.")

        elif key in (ord('s'), ord('S')):
            H_to_save = H_pix2robot_locked if calibration_locked else H_pix2robot
            if H_to_save is not None:
                save_calibration(H_to_save, last_reproj_err_px)

        elif key in (ord('d'), ord('D')):
            H_l, H_inv_l = load_calibration()
            if H_l is not None:
                H_pix2robot_locked = H_l
                H_robot2pix_locked = H_inv_l
                calibration_locked = True

        elif key in (ord('t'), ord('T')):
            USE_CORNERS = not USE_CORNERS
            print(f"Mode -> "
                  f"{'CORNERS (16pt)' if USE_CORNERS else 'CENTRES (4pt)'}")

        elif key in (ord('v'), ord('V')):
            H_a, _ = get_active_H()
            if H_a is None:
                print("No calibration.")
            else:
                print("\n── VERIFY ──")
                s = unit_scale()
                worst = (None, 0.0)
                for mid in sorted(WORLD_MARKERS):
                    if len(marker_corner_history[mid]) < 3:
                        print(f"  id {mid}: not enough samples")
                        continue
                    avg = np.median(np.stack(marker_corner_history[mid]), axis=0)
                    centre_pix = avg.mean(axis=0)
                    meas_c = apply_H(H_a, [centre_pix])[0]
                    exp_c  = WORLD_MARKERS[mid][:2]
                    e_mm = np.linalg.norm(meas_c - exp_c) * 1000.0
                    print(f"  id {mid}: err={e_mm:5.2f} mm")
                    if e_mm > worst[1]:
                        worst = (mid, e_mm)
                if worst[0] is not None:
                    print(f"  ► WORST: id {worst[0]} = {worst[1]:.2f} mm")

        elif key in (ord('g'), ord('G')):
            SHOW_GRID = not SHOW_GRID

        elif key in (ord('u'), ord('U')):
            order = ["mm", "cm", "m"]
            COORD_UNITS = order[(order.index(COORD_UNITS) + 1) % len(order)]
            print(f"Units -> {COORD_UNITS}")

        elif key in (ord('x'), ord('X')):
            if latest_color_image is not None:
                run_sam3_detect(latest_color_image.copy(),
                                latest_depth_image)

        elif key in (ord('p'), ord('P')):
            run_sequence(SEQUENCE_FILE)

        elif key in (ord('m'), ord('M')):
            run_grid_poses()

        elif ord('1') <= key <= ord('9'):
            idx = key - ord('1')
            if idx < len(PART_KEYS):
                ACTIVE_PART = PART_KEYS[idx]
                print(f"Active part -> {PARTS[ACTIVE_PART]['name']}")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    try:
        ROBOT.stop()
    except Exception:
        pass
