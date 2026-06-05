#!/usr/bin/python3
"""
calib_pick.py — RealSense world calibration + SAM3 pick planner in one loop.

Flow:
  1. SAM3 model is loaded ONCE at startup (slow, ~tens of seconds).
  2. Calibration window runs normally (ArUco / Kabsch / locking / etc.).
  3. Press  X   -> grab the current color frame, run SAM3 pick planner on it,
                   print the best pick (pixel + world XYZ), save annotated img.
  4. Press it again any time — model stays loaded, picks are fast (~1s).
  5. ESC quits.

Keys (added):
  X = capture frame and run SAM3 pick

Everything else from the original calibration script is kept.
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from collections import deque
import json
import os
import time

# ─── SAM3 / pick_planner imports ─────────────────────────────────────────────
import torch
from scipy.ndimage import distance_transform_edt, binary_erosion, binary_dilation
from PIL import Image, ImageDraw
from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor

# ============================================================
# SETTINGS  (unchanged from your calibration script)
# ============================================================

MARKER_SIZE = 0.037  # m
HALF = MARKER_SIZE / 2

DX_15 = 0.173
DY_13 = 0.263

WORLD_MARKERS = {
    1: np.array([HALF,         HALF,         0.0]),
    5: np.array([DX_15 + HALF, HALF,         0.0]),
    0: np.array([HALF,         DY_13 + HALF, 0.0]),
    2: np.array([DX_15 + HALF, DY_13 + HALF, 0.0]),
}
EXPECTED_IDS = set(WORLD_MARKERS.keys())

WORKSPACE_W = DX_15 + MARKER_SIZE
WORKSPACE_H = DY_13 + MARKER_SIZE

COLOR_W, COLOR_H = 1280, 720
DEPTH_W, DEPTH_H = 1280, 720
FPS = 30
UI_SCALE = COLOR_W / 640.0

_p = int(round(7 * UI_SCALE))
DEPTH_PATCH_SIZE     = _p if _p % 2 == 1 else _p + 1
MAX_FIT_ERR_M        = 0.010
MAX_REPROJ_ERR_PX    = 2.0 * UI_SCALE
MAX_SIZE_RATIO_DEV   = 0.05
AVG_WINDOW           = 30
CALIB_FILE           = "world_calib.json"
KABSCH_SOURCE        = 'ippe'

# ─── SAM3 pick-planner config ────────────────────────────────────────────────
SAM3_PROMPT          = "black objects"
SAM3_SCORE_THRESHOLD = 0.8
CAPTURE_PATH         = "captured.png"
PICK_OUTPUT_PATH     = "output_pick.jpg"

GRIPPER_FINGER_PAD   = 20
GRIPPER_BODY_RADIUS  = 14
MIN_FINGER_CLEARANCE = 10

W_FREE_PERIM   = 0.45
W_FINGER_CLR   = 0.30
W_CORNER_CLR   = 0.05
W_CONFIDENCE   = 0.05
W_BORDER       = 0.10
W_AREA_SANITY  = 0.05

AREA_RATIO_MIN = 0.60
AREA_RATIO_MAX = 1.40

USE_ORIENTATION_FILTER = True
MIN_COMPACTNESS = 0.55
MAX_BBOX_ASPECT = 2.0
DRAW_MASK_OVERLAY = True

# ============================================================
# REALSENSE + DEPTH FILTERS
# ============================================================

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, FPS)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 2)
spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
spatial.set_option(rs.option.filter_smooth_delta, 20)

temporal = rs.temporal_filter()
temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
temporal.set_option(rs.option.filter_smooth_delta, 20)

hole_filling = rs.hole_filling_filter(1)

intr = (
    profile.get_stream(rs.stream.color)
    .as_video_stream_profile()
    .get_intrinsics()
)

camera_matrix = np.array([
    [intr.fx, 0,       intr.ppx],
    [0,       intr.fy, intr.ppy],
    [0,       0,       1]
], dtype=np.float64)
dist_coeffs = np.array(intr.coeffs, dtype=np.float64)

print(f"Stream: {intr.width}x{intr.height}  "
      f"fx={intr.fx:.1f} fy={intr.fy:.1f}  "
      f"ppx={intr.ppx:.1f} ppy={intr.ppy:.1f}  "
      f"UI_SCALE={UI_SCALE:.2f}  patch={DEPTH_PATCH_SIZE}")

# ============================================================
# ARUCO
# ============================================================

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_params.cornerRefinementMethod        = cv2.aruco.CORNER_REFINE_SUBPIX
aruco_params.cornerRefinementWinSize       = 5
aruco_params.cornerRefinementMaxIterations = 50
aruco_params.cornerRefinementMinAccuracy   = 0.01
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

marker_corners_local = np.array([
    [-HALF,  HALF, 0],
    [ HALF,  HALF, 0],
    [ HALF, -HALF, 0],
    [-HALF, -HALF, 0],
], dtype=np.float64)

# ============================================================
# GLOBALS
# ============================================================

T_world_cam        = None
T_world_cam_locked = None
calibration_locked = False
latest_depth_image = None
latest_color_image = None
clicked_points     = []
marker_history     = {mid: deque(maxlen=AVG_WINDOW) for mid in WORLD_MARKERS}

# ============================================================
# SAM3 — load ONCE
# ============================================================

print("\n══════════════════════════════════════════════")
print(" Loading SAM3 model (this happens only once)…")
print("══════════════════════════════════════════════")
_t0 = time.time()
_sam3_model     = build_sam3_image_model().float()
_sam3_processor = Sam3Processor(_sam3_model)
print(f" SAM3 ready in {time.time()-_t0:.1f}s\n")


# ============================================================
# PICK-PLANNER HELPERS  (verbatim from pick_planner.py)
# ============================================================

def mask_to_bool(mask):
    m = mask.cpu().numpy()
    if m.ndim == 3:
        m = m.squeeze(0)
    return m.astype(bool)

def largest_inscribed_circle(bool_mask):
    dt = distance_transform_edt(bool_mask)
    cy, cx = np.unravel_index(dt.argmax(), dt.shape)
    return int(cx), int(cy), float(dt[cy, cx])

def principal_axes(bool_mask):
    ys, xs = np.where(bool_mask)
    if len(xs) < 5:
        return 0.0, np.array([1., 0.]), np.array([0., 1.])
    pts = np.column_stack([xs - xs.mean(), ys - ys.mean()]).astype(float)
    cov = pts.T @ pts / len(pts)
    vals, vecs = np.linalg.eigh(cov)
    major = vecs[:, 1]
    minor = np.array([-major[1], major[0]])
    angle = float(np.degrees(np.arctan2(major[1], major[0])))
    return angle, major, minor

def oriented_bbox(bool_mask, major, minor):
    ys, xs = np.where(bool_mask)
    if len(xs) == 0:
        return None
    cx_m, cy_m = xs.mean(), ys.mean()
    rel = np.column_stack([xs - cx_m, ys - cy_m])
    proj_major = rel @ major
    proj_minor = rel @ minor
    hw_major = (proj_major.max() - proj_major.min()) / 2.0
    hw_minor = (proj_minor.max() - proj_minor.min()) / 2.0
    centre = np.array([cx_m, cy_m])
    corners = []
    for sa in (-1, 1):
        for sb in (-1, 1):
            corners.append(centre + sa * hw_major * major + sb * hw_minor * minor)
    return centre, hw_major, hw_minor, np.array(corners)

def compactness_score(bool_mask):
    eroded   = binary_erosion(bool_mask, iterations=1)
    boundary = bool_mask ^ eroded
    perim    = boundary.sum()
    area     = bool_mask.sum()
    return float(4 * np.pi * area / (perim ** 2)) if perim else 0.0

def border_clearance(bool_mask, h, w, frac=0.03):
    my, mx = int(frac * h), int(frac * w)
    interior = np.zeros_like(bool_mask)
    interior[my:h-my, mx:w-mx] = True
    tot = bool_mask.sum()
    return float((bool_mask & interior).sum() / tot) if tot else 0.0

def bbox_aspect_ratio(box):
    x0, y0, x1, y1 = box.tolist()
    w = max(x1 - x0, 1); h = max(y1 - y0, 1)
    r = w / h
    return float(max(r, 1.0 / r))

def sample_clearance(free_dt, x, y):
    h, w = free_dt.shape
    x = int(np.clip(round(x), 0, w - 1))
    y = int(np.clip(round(y), 0, h - 1))
    return float(free_dt[y, x])

def build_freespace(bool_masks, img_h, img_w):
    occupied = np.zeros((img_h, img_w), dtype=bool)
    for m in bool_masks:
        occupied |= m
    free_dt = distance_transform_edt(~occupied)
    return occupied, free_dt

def free_perimeter_ratio(bool_mask, occupied, dilate_iter=2):
    dilated = binary_dilation(bool_mask, iterations=dilate_iter)
    ring    = dilated & ~bool_mask
    if ring.sum() == 0:
        return 0.0
    others_occ = occupied & ~bool_mask
    free_ring  = ring & ~others_occ
    return float(free_ring.sum() / ring.sum())

def gripper_finger_clearance(free_dt, centre, minor, hw_minor):
    reach = hw_minor + GRIPPER_FINGER_PAD
    p1 = centre + reach * minor
    p2 = centre - reach * minor
    c1 = sample_clearance(free_dt, p1[0], p1[1])
    c2 = sample_clearance(free_dt, p2[0], p2[1])
    return min(c1, c2), 0.5 * (c1 + c2), (p1, p2)

def corner_clearance(free_dt, corners):
    vals = [sample_clearance(free_dt, c[0], c[1]) for c in corners]
    return float(min(vals)), vals

def filter_orientation(masks, boxes, scores):
    keep = []
    for i in range(len(scores)):
        bm = mask_to_bool(masks[i])
        c  = compactness_score(bm)
        a  = bbox_aspect_ratio(boxes[i])
        if c >= MIN_COMPACTNESS and a <= MAX_BBOX_ASPECT:
            keep.append(i)
    if not keep:
        return None, None, None
    idx = torch.tensor(keep)
    return masks[idx], boxes[idx], scores[idx]

def filter_area_sanity(masks, boxes, scores):
    areas = np.array([mask_to_bool(m).sum() for m in masks], dtype=float)
    if len(areas) < 2:
        return masks, boxes, scores, areas
    median = np.median(areas)
    keep = []
    for i, a in enumerate(areas):
        r = a / median
        if AREA_RATIO_MIN <= r <= AREA_RATIO_MAX:
            keep.append(i)
    if not keep:
        return None, None, None, areas
    idx = torch.tensor(keep)
    return masks[idx], boxes[idx], scores[idx], areas[keep]

def compute_pick_scores(masks, boxes, scores, img_h, img_w):
    N = len(scores)
    conf = scores.cpu().numpy()
    bool_masks = [mask_to_bool(masks[i]) for i in range(N)]
    occupied, free_dt = build_freespace(bool_masks, img_h, img_w)

    pick_points, angles  = [], []
    free_perim, fing_min, fing_mean, corner_min = [], [], [], []
    finger_pts_all, corner_pts_all, areas = [], [], []

    for bm in bool_masks:
        cx, cy, r_ic = largest_inscribed_circle(bm)
        pick_points.append((cx, cy, r_ic))
        ang, major, minor = principal_axes(bm)
        angles.append(ang)
        centre, hw_maj, hw_min, corners = oriented_bbox(bm, major, minor)
        corner_pts_all.append(corners)
        fmin, fmean, fpts = gripper_finger_clearance(free_dt, centre, minor, hw_min)
        fing_min.append(fmin); fing_mean.append(fmean); finger_pts_all.append(fpts)
        cmin, _ = corner_clearance(free_dt, corners)
        corner_min.append(cmin)
        free_perim.append(free_perimeter_ratio(bm, occupied))
        areas.append(bm.sum())

    free_perim = np.array(free_perim); fing_min = np.array(fing_min)
    fing_mean = np.array(fing_mean);   corner_min = np.array(corner_min)
    areas = np.array(areas, dtype=float)

    median_area = np.median(areas) if len(areas) else 1.0
    area_dev    = np.abs(areas - median_area) / max(median_area, 1.0)
    area_sanity = np.clip(1.0 - area_dev, 0.0, 1.0)
    border_raw  = np.array([border_clearance(bm, img_h, img_w) for bm in bool_masks])

    def norm(a):
        rng = a.max() - a.min()
        return (a - a.min()) / rng if rng > 0 else np.ones_like(a)

    pick_scores = (
        W_FREE_PERIM  * norm(free_perim) +
        W_FINGER_CLR  * norm(fing_min)   +
        W_CORNER_CLR  * norm(corner_min) +
        W_CONFIDENCE  * norm(conf)       +
        W_BORDER      * norm(border_raw) +
        W_AREA_SANITY * area_sanity
    )
    vetoed = fing_min < MIN_FINGER_CLEARANCE
    if vetoed.any():
        pick_scores = np.where(vetoed, 0.0, pick_scores)

    raw = {
        "confidence": conf, "free_perim": free_perim,
        "finger_min": fing_min, "finger_mean": fing_mean,
        "corner_min": corner_min, "area": areas,
        "border": border_raw, "angle": np.array(angles), "vetoed": vetoed,
    }
    return pick_scores, pick_points, angles, raw, finger_pts_all, corner_pts_all

PALETTE = [(255,80,80,90),(80,200,120,90),(80,140,255,90),
           (255,200,50,90),(200,80,255,90),(50,220,220,90)]

def draw_results(image, masks, boxes, pick_scores, pick_points, angles,
                 raw, finger_pts_all, corner_pts_all, best_idx):
    if DRAW_MASK_OVERLAY:
        overlay = Image.new("RGBA", image.size, (0,0,0,0))
        for i, m in enumerate(masks):
            bm = mask_to_bool(m)
            layer = np.zeros((*bm.shape, 4), dtype=np.uint8)
            layer[bm] = PALETTE[i % len(PALETTE)]
            overlay = Image.alpha_composite(overlay, Image.fromarray(layer, "RGBA"))
        image = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")
    draw = ImageDraw.Draw(image)

    for i in range(len(pick_scores)):
        cx, cy, r_ic = pick_points[i]
        x0, y0, x1, y1 = [int(v) for v in boxes[i].tolist()]
        is_best = (i == best_idx)
        vetoed  = bool(raw["vetoed"][i])
        col = "lime" if is_best else ("#ff5050" if vetoed else "#aaaaaa")
        w   = 4 if is_best else 1
        draw.rectangle([x0,y0,x1,y1], outline=col, width=w)
        ri = int(r_ic)
        draw.ellipse([cx-ri, cy-ri, cx+ri, cy+ri], outline=col, width=2)
        draw.ellipse([cx-3, cy-3, cx+3, cy+3], fill="yellow")
        ang = np.radians(angles[i]); L = ri + 16
        ex, ey = cx + L*np.cos(ang), cy + L*np.sin(ang)
        draw.line([cx, cy, int(ex), int(ey)], fill="cyan", width=2)
        draw.line([cx, cy, int(2*cx-ex), int(2*cy-ey)], fill="cyan", width=2)
        p1, p2 = finger_pts_all[i]
        fc = "red" if vetoed else "orange"
        for p in (p1, p2):
            px, py = int(p[0]), int(p[1])
            draw.ellipse([px-5, py-5, px+5, py+5], outline=fc, width=2)
        for c in corner_pts_all[i]:
            px, py = int(c[0]), int(c[1])
            draw.ellipse([px-2, py-2, px+2, py+2], fill="magenta")
        label = (f"#{i+1} {pick_scores[i]:.2f} "
                 f"fp={raw['free_perim'][i]:.2f} "
                 f"fc={raw['finger_min'][i]:.0f}px "
                 f"cc={raw['corner_min'][i]:.0f}px")
        draw.text((x0, max(0, y0-14)), label, fill=col)

    cx, cy, _ = pick_points[best_idx]
    r = 22
    draw.ellipse([cx-r, cy-r, cx+r, cy+r], outline="red", width=3)
    draw.line([cx-2*r, cy, cx+2*r, cy], fill="red", width=2)
    draw.line([cx, cy-2*r, cx, cy+2*r], fill="red", width=2)
    draw.text((cx+r+4, cy-8), "PICK", fill="red")
    return image


# ============================================================
# CALIBRATION HELPERS  (unchanged)
# ============================================================

def get_depth_median(depth_image, u, v, size=DEPTH_PATCH_SIZE):
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

def kabsch(P, Q):
    cP = P.mean(axis=0); cQ = Q.mean(axis=0)
    Pc = P - cP; Qc = Q - cQ
    H = Pc.T @ Qc
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = cQ - R @ cP
    pred = (R @ P.T).T + t
    per_pt = np.linalg.norm(pred - Q, axis=1)
    return R, t, float(per_pt.mean()), per_pt

def get_active_T():
    return T_world_cam_locked if calibration_locked else T_world_cam

def pixel_to_world(u, v, depth_image):
    T = get_active_T()
    if T is None:
        return None
    depth = get_depth_median(depth_image, u, v)
    if depth <= 0:
        return None
    pt_cam = rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], depth)
    pt_h = np.array([pt_cam[0], pt_cam[1], pt_cam[2], 1.0])
    return (T @ pt_h)[:3]

def in_workspace(world_xyz):
    x, y = world_xyz[0], world_xyz[1]
    return (0.0 <= x <= WORKSPACE_W) and (0.0 <= y <= WORKSPACE_H)

def save_calibration(T_wc, residual_m):
    data = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "T_world_cam": T_wc.tolist(),
        "marker_size_m": MARKER_SIZE,
        "world_markers": {str(k): v.tolist() for k, v in WORLD_MARKERS.items()},
        "kabsch_residual_mm": (residual_m * 1000.0) if residual_m is not None else None,
        "intrinsics": {
            "fx": intr.fx, "fy": intr.fy,
            "ppx": intr.ppx, "ppy": intr.ppy,
            "width": intr.width, "height": intr.height,
        },
    }
    with open(CALIB_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Calibration saved -> {CALIB_FILE}")

def load_calibration():
    if not os.path.exists(CALIB_FILE):
        print(f"Geen calibratie bestand gevonden: {CALIB_FILE}")
        return None
    with open(CALIB_FILE, "r") as f:
        data = json.load(f)
    T = np.array(data["T_world_cam"], dtype=np.float64)
    saved_intr = data.get("intrinsics", {})
    if abs(saved_intr.get("fx", intr.fx) - intr.fx) > 1.0:
        print("WARN: andere camera intrinsics in calibratiebestand.")
    print(f"Calibration loaded  (residual={data.get('kabsch_residual_mm','?')} mm)")
    return T


# ============================================================
# THE SAM3 PICK ACTION (called on 'X')
# ============================================================

def run_sam3_pick(bgr_frame, depth_image):
    """Capture-> SAM3 -> compute best pick -> print pixel + world XYZ."""
    print("\n" + "═"*60)
    print(" SAM3 PICK — running on current frame")
    print("═"*60)

    # Save + open as PIL (SAM3 processor expects PIL)
    cv2.imwrite(CAPTURE_PATH, bgr_frame)
    image = Image.open(CAPTURE_PATH).convert("RGB")
    img_w, img_h = image.size
    print(f"  Captured: {CAPTURE_PATH}  ({img_w}x{img_h})")
    print(f"  Prompt:   {SAM3_PROMPT!r}")

    t0 = time.time()
    try:
        with torch.autocast(device_type="cuda", dtype=torch.float32):
            state  = _sam3_processor.set_image(image)
            output = _sam3_processor.set_text_prompt(state=state, prompt=SAM3_PROMPT)
    except Exception as e:
        print(f"  SAM3 failed: {e}")
        return

    masks, boxes, scores = output["masks"], output["boxes"], output["scores"]
    keep = scores >= SAM3_SCORE_THRESHOLD
    masks, boxes, scores = masks[keep], boxes[keep], scores[keep]
    if len(scores) == 0:
        print("  No detections above threshold.")
        return
    print(f"  Detected {len(scores)} instance(s) in {time.time()-t0:.1f}s")

    if USE_ORIENTATION_FILTER:
        masks, boxes, scores = filter_orientation(masks, boxes, scores)
        if masks is None:
            print("  Nothing passed orientation filter."); return

    masks, boxes, scores, _ = filter_area_sanity(masks, boxes, scores)
    if masks is None:
        print("  Nothing passed area-sanity filter."); return

    (pick_scores, pick_points, angles, raw,
     finger_pts, corner_pts) = compute_pick_scores(
        masks, boxes, scores, img_h, img_w)

    if pick_scores.max() == 0:
        print("  All candidates vetoed — no safe pick."); return

    best = int(pick_scores.argmax())
    cx, cy, r_ic = pick_points[best]

    # World coordinates
    world = pixel_to_world(cx, cy, depth_image)
    T_active = get_active_T()
    print(f"\n  ── BEST PICK ──")
    print(f"   instance #     : {best+1}")
    print(f"   pixel          : ({cx}, {cy})")
    print(f"   inscribed r    : {r_ic:.1f} px")
    print(f"   gripper angle  : {angles[best]:.1f}°")
    print(f"   pick score     : {pick_scores[best]:.3f}")
    print(f"   free perimeter : {raw['free_perim'][best]:.2%}")
    print(f"   finger clr min : {raw['finger_min'][best]:.1f} px")
    if T_active is None:
        print(f"   WORLD XYZ      : (no calibration active)")
    elif world is None:
        print(f"   WORLD XYZ      : (no depth at pick pixel)")
    else:
        ws = " [in workspace]" if in_workspace(world) else " [OUT OF workspace]"
        print(f"   WORLD XYZ      : X={world[0]*100:6.1f}  "
              f"Y={world[1]*100:6.1f}  Z={world[2]*100:6.1f} cm{ws}")

    annotated = draw_results(image.copy(), masks, boxes,
                             pick_scores, pick_points, angles, raw,
                             finger_pts, corner_pts, best)
    annotated.save(PICK_OUTPUT_PATH)
    print(f"   annotated img  : {PICK_OUTPUT_PATH}")
    print("═"*60 + "\n")

    # Show annotated result in a separate window
    ann_bgr = cv2.cvtColor(np.array(annotated), cv2.COLOR_RGB2BGR)
    cv2.namedWindow("SAM3 Pick Result", cv2.WINDOW_NORMAL)
    cv2.imshow("SAM3 Pick Result", ann_bgr)


# ============================================================
# MOUSE
# ============================================================

def mouse_callback(event, x, y, flags, param):
    global clicked_points
    if event == cv2.EVENT_LBUTTONDOWN:
        if latest_depth_image is None or get_active_T() is None:
            print("Geen geldige calibratie.")
            return
        world = pixel_to_world(x, y, latest_depth_image)
        if world is None:
            print(f"Geen depth op ({x},{y}).")
            return
        d = get_depth_median(latest_depth_image, x, y)
        ws_flag = "" if in_workspace(world) else "  [BUITEN workspace]"
        print(f"Pixel ({x},{y}) depth={d*100:5.1f} cm  ->  "
              f"X={world[0]*100:6.1f}  Y={world[1]*100:6.1f}  "
              f"Z={world[2]*100:6.1f}  cm{ws_flag}")
        clicked_points.append(((x, y), world))
        if len(clicked_points) > 10:
            clicked_points.pop(0)
    elif event == cv2.EVENT_RBUTTONDOWN:
        clicked_points = []
        print("Punten gewist.")


cv2.namedWindow("World Calibration", cv2.WINDOW_NORMAL)
cv2.resizeWindow("World Calibration", COLOR_W, COLOR_H)
cv2.setMouseCallback("World Calibration", mouse_callback)

# UI scaled sizes
S_BIG    = 0.55 * UI_SCALE
S_MED    = 0.50 * UI_SCALE
S_SMALL  = 0.42 * UI_SCALE
S_FOOT   = 0.45 * UI_SCALE
T_BOLD   = max(1, int(round(2 * UI_SCALE * 0.7)))
T_THIN   = max(1, int(round(1 * UI_SCALE * 0.7)))
LINE_H   = int(round(22 * UI_SCALE))
LINE_HS  = int(round(15 * UI_SCALE))

# ============================================================
# MAIN LOOP
# ============================================================

print("L=test  R=wis  SPACE=lock/unlock  C=clear hist  "
      "S=save  D=load  X=SAM3 pick  ESC=stop")

_loaded = load_calibration()
if _loaded is not None:
    T_world_cam_locked = _loaded
    calibration_locked = True
    print("Auto-locked using saved calibration. SPACE to unlock & recalibrate.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        df = spatial.process(depth_frame)
        df = temporal.process(df)
        df = hole_filling.process(df)
        depth_frame_f = df.as_depth_frame()

        latest_depth_image = np.asanyarray(depth_frame_f.get_data())
        latest_color_image = np.asanyarray(color_frame.get_data())
        img = latest_color_image.copy()

        corners, ids, _ = detector.detectMarkers(img)

        diag_lines, size_lines, warn_lines = [], [], []
        fit_ok = z_ok = False
        residual_m = worst_residual_m = None
        cam_pos_world = None
        T_cam_world_show = None
        ids_for_kabsch = []

        centers_depth, centers_ippe = {}, {}
        pixel_centers, reproj_err = {}, {}
        seen_ids = set()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)

            for mid_arr, m_corner in zip(ids.flatten(), corners):
                mid = int(mid_arr)
                seen_ids.add(mid)
                if mid not in WORLD_MARKERS:
                    continue
                img_pts = m_corner.reshape(4, 2).astype(np.float64)
                cx_f, cy_f = img_pts.mean(axis=0)
                pixel_centers[mid] = (cx_f, cy_f)

                depth = get_depth_median(latest_depth_image, cx_f, cy_f)
                if depth > 0:
                    pt_cam = rs.rs2_deproject_pixel_to_point(intr, [cx_f, cy_f], depth)
                    centers_depth[mid] = np.array(pt_cam, dtype=np.float64)

                ok, rvec, tvec = cv2.solvePnP(
                    marker_corners_local, img_pts,
                    camera_matrix, dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if ok:
                    centers_ippe[mid] = tvec.flatten()

            unexpected = seen_ids - EXPECTED_IDS
            missing = EXPECTED_IDS - seen_ids
            if unexpected:
                warn_lines.append(f"Onverwachte ID(s): {sorted(unexpected)}")
            if missing:
                warn_lines.append(f"Ontbrekende ID(s): {sorted(missing)}")

            for mid in sorted(centers_depth.keys() & centers_ippe.keys()):
                d_dep = float(np.linalg.norm(centers_depth[mid]))
                d_ipp = float(np.linalg.norm(centers_ippe[mid]))
                if d_dep > 0.05 and d_ipp > 0.05:
                    ratio = d_ipp / d_dep
                    if abs(ratio - 1.0) > MAX_SIZE_RATIO_DEV:
                        suggested = MARKER_SIZE * d_dep / d_ipp
                        size_lines.append(
                            f"M{mid}: IPPE/depth={ratio:.2f}  "
                            f"MARKER_SIZE ~ {suggested*1000:.1f} mm"
                        )

            for mid in pixel_centers:
                if mid not in WORLD_MARKERS:
                    continue
                cx, cy = pixel_centers[mid]
                lines = []
                if mid in centers_depth:
                    d_dep = float(np.linalg.norm(centers_depth[mid])) * 100
                    line = f"dpt {d_dep:.0f}"
                    if mid in centers_ippe:
                        d_ipp = float(np.linalg.norm(centers_ippe[mid])) * 100
                        line += f" | ipp {d_ipp:.0f} cm"
                    else:
                        line += " cm"
                    lines.append(line)
                ex = WORLD_MARKERS[mid][0] * 100
                ey = WORLD_MARKERS[mid][1] * 100
                lines.append(f"verw ({ex:.1f},{ey:.1f}) cm")
                ox = int(cx) - int(60 * UI_SCALE)
                oy0 = int(cy) + int(30 * UI_SCALE)
                for i, txt in enumerate(lines):
                    cv2.putText(img, txt,
                        (ox, oy0 + i * int(18 * UI_SCALE)),
                        cv2.FONT_HERSHEY_SIMPLEX, S_SMALL,
                        (0, 255, 255) if i == 0 else (100, 165, 255),
                        T_THIN, cv2.LINE_AA)

            if KABSCH_SOURCE == 'ippe':
                for mid in centers_ippe:
                    if mid in WORLD_MARKERS:
                        marker_history[mid].append(centers_ippe[mid])
            else:
                for mid in centers_depth:
                    marker_history[mid].append(centers_depth[mid])

            ids_known = sorted(m for m in centers_depth if m in WORLD_MARKERS)
            for i in range(len(ids_known)):
                for j in range(i + 1, len(ids_known)):
                    a, b = ids_known[i], ids_known[j]
                    meas = np.linalg.norm(centers_depth[a] - centers_depth[b])
                    exp_ = np.linalg.norm(WORLD_MARKERS[a] - WORLD_MARKERS[b])
                    diff_cm = (meas - exp_) * 100
                    diag_lines.append(
                        f"d({a}-{b}): meet {meas*100:5.1f}  "
                        f"verw {exp_*100:5.1f}  ({diff_cm:+.1f}) cm"
                    )

        ids_for_kabsch = sorted(
            m for m in marker_history
            if len(marker_history[m]) >= 3 and m in WORLD_MARKERS
        )

        if len(ids_for_kabsch) >= 3:
            P = np.array([WORLD_MARKERS[m] for m in ids_for_kabsch], dtype=np.float64)
            Q = np.array([
                np.median(np.stack(marker_history[m]), axis=0)
                for m in ids_for_kabsch
            ], dtype=np.float64)

            R, t, residual_m, per_pt = kabsch(P, Q)
            worst_residual_m = float(per_pt.max())

            T_cam_world_new = np.eye(4)
            T_cam_world_new[:3, :3] = R
            T_cam_world_new[:3, 3] = t
            T_world_cam_new = np.linalg.inv(T_cam_world_new)
            cam_pos_world = T_world_cam_new[:3, 3]

            fit_ok = residual_m < MAX_FIT_ERR_M
            z_ok = cam_pos_world[2] > 0

            if fit_ok and z_ok:
                T_cam_world_show = T_cam_world_new
                if not calibration_locked:
                    T_world_cam = T_world_cam_new

            if T_cam_world_show is not None:
                rvec_w, _ = cv2.Rodrigues(T_cam_world_show[:3, :3])
                tvec_w = T_cam_world_show[:3, 3]
                for mid in ids_for_kabsch:
                    if mid in pixel_centers:
                        proj, _ = cv2.projectPoints(
                            WORLD_MARKERS[mid].reshape(1, 3),
                            rvec_w, tvec_w, camera_matrix, dist_coeffs)
                        px_exp, py_exp = proj[0, 0]
                        px_obs, py_obs = pixel_centers[mid]
                        err = float(np.hypot(px_exp - px_obs, py_exp - py_obs))
                        reproj_err[mid] = err
                        col_line = (0, 255, 0) if err < MAX_REPROJ_ERR_PX else (0, 165, 255)
                        cv2.line(img, (int(px_exp), int(py_exp)),
                                 (int(px_obs), int(py_obs)),
                                 col_line, T_THIN, cv2.LINE_AA)
                        cv2.circle(img, (int(px_exp), int(py_exp)),
                                   int(4 * UI_SCALE), col_line, 1, cv2.LINE_AA)

        T_cam_world_for_viz = None
        if calibration_locked and T_world_cam_locked is not None:
            T_cam_world_for_viz = np.linalg.inv(T_world_cam_locked)
        elif T_cam_world_show is not None:
            T_cam_world_for_viz = T_cam_world_show

        if T_cam_world_for_viz is not None:
            R_cw = T_cam_world_for_viz[:3, :3]
            t_cw = T_cam_world_for_viz[:3, 3]
            rvec_w, _ = cv2.Rodrigues(R_cw)
            cv2.drawFrameAxes(img, camera_matrix, dist_coeffs,
                              rvec_w, t_cw.reshape(3, 1), 0.05, T_BOLD)

            axis_pts = np.array([
                [0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.05]], dtype=np.float64)
            axis_img, _ = cv2.projectPoints(axis_pts, rvec_w, t_cw,
                                            camera_matrix, dist_coeffs)
            for lbl, p, col in zip(["X","Y","Z"], axis_img.reshape(-1, 2),
                                   [(0,0,255),(0,255,0),(255,0,0)]):
                cv2.putText(img, lbl, (int(p[0]) + 6, int(p[1]) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, S_MED, col, T_BOLD, cv2.LINE_AA)

            ws_world = np.array([
                [0.0, 0.0, 0.0], [WORKSPACE_W, 0.0, 0.0],
                [WORKSPACE_W, WORKSPACE_H, 0.0], [0.0, WORKSPACE_H, 0.0],
            ], dtype=np.float64)
            ws_img, _ = cv2.projectPoints(ws_world, rvec_w, t_cw,
                                          camera_matrix, dist_coeffs)
            ws_pts = ws_img.reshape(-1, 2).astype(np.int32)
            cv2.polylines(img, [ws_pts], isClosed=True,
                          color=(255, 0, 255), thickness=T_THIN, lineType=cv2.LINE_AA)

            origin_img, _ = cv2.projectPoints(
                np.array([[0.0, 0.0, 0.0]]), rvec_w, t_cw,
                camera_matrix, dist_coeffs)
            ox, oy = origin_img[0, 0].astype(int)
            cv2.drawMarker(img, (ox, oy), (255, 0, 255),
                           cv2.MARKER_CROSS, int(18 * UI_SCALE), T_BOLD, cv2.LINE_AA)
            cv2.putText(img, "(0,0,0)", (ox + 10, oy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, S_MED,
                        (255, 0, 255), T_BOLD, cv2.LINE_AA)

        for (px, py), world in clicked_points:
            ws = in_workspace(world)
            col_dot = (0, 0, 255) if ws else (0, 165, 255)
            cv2.circle(img, (px, py), int(6 * UI_SCALE), col_dot, -1)
            cv2.circle(img, (px, py), int(7 * UI_SCALE), (255, 255, 255), 1)
            flag = "" if ws else " !"
            label = (f"X={world[0]*100:.1f} Y={world[1]*100:.1f} "
                     f"Z={world[2]*100:.1f}{flag}")
            cv2.putText(img, label, (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, S_MED,
                        col_dot, T_THIN, cv2.LINE_AA)

        if len(clicked_points) >= 2:
            (p1, w1) = clicked_points[-2]
            (p2, w2) = clicked_points[-1]
            cv2.line(img, p1, p2, (0, 255, 255), T_THIN, cv2.LINE_AA)
            dist_cm = float(np.linalg.norm(np.array(w1) - np.array(w2))) * 100
            mx, my = (p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2
            cv2.putText(img, f"{dist_cm:.1f} cm", (mx + 8, my),
                        cv2.FONT_HERSHEY_SIMPLEX, S_MED,
                        (0, 255, 255), T_BOLD, cv2.LINE_AA)

        y0 = int(28 * UI_SCALE)
        if cam_pos_world is not None:
            col = (0, 255, 0) if (fit_ok and z_ok) else (0, 0, 255)
            cv2.putText(img,
                f"Cam in wereld: X={cam_pos_world[0]*100:5.1f} "
                f"Y={cam_pos_world[1]*100:5.1f} "
                f"Z={cam_pos_world[2]*100:5.1f} cm",
                (10, y0), cv2.FONT_HERSHEY_SIMPLEX,
                S_BIG, col, T_BOLD, cv2.LINE_AA)
            y0 += LINE_H
            n_avg_min = min((len(marker_history[mid]) for mid in ids_for_kabsch),
                            default=0)
            lock_label = "[LOCKED]" if calibration_locked else "[live]"
            worst_str = (f"  worst={worst_residual_m*1000:4.1f}mm"
                         if worst_residual_m is not None else "")
            cv2.putText(img,
                f"Kabsch fit: {residual_m*1000:5.1f} mm{worst_str}   "
                f"n={n_avg_min}/{AVG_WINDOW}   {lock_label}",
                (10, y0), cv2.FONT_HERSHEY_SIMPLEX, S_MED,
                (255, 0, 255) if calibration_locked else col,
                T_THIN, cv2.LINE_AA)
            y0 += LINE_HS + 5
        elif calibration_locked:
            cv2.putText(img, "[LOCKED] (geen live markers gevonden)",
                (10, y0), cv2.FONT_HERSHEY_SIMPLEX,
                S_BIG, (255, 0, 255), T_BOLD, cv2.LINE_AA)
            y0 += LINE_H

        if reproj_err:
            err_str = "  ".join(f"M{m}:{e:.1f}px"
                                for m, e in sorted(reproj_err.items()))
            max_err = max(reproj_err.values())
            col_re = (0, 255, 0) if max_err < MAX_REPROJ_ERR_PX else (0, 165, 255)
            cv2.putText(img, f"Reproj: {err_str}",
                (10, y0), cv2.FONT_HERSHEY_SIMPLEX,
                S_SMALL, col_re, T_THIN, cv2.LINE_AA)
            y0 += LINE_HS

        for line in diag_lines:
            cv2.putText(img, line, (10, y0),
                cv2.FONT_HERSHEY_SIMPLEX, S_SMALL,
                (255, 255, 0), T_THIN, cv2.LINE_AA)
            y0 += LINE_HS

        if size_lines:
            y0 += 4
            for line in size_lines:
                cv2.putText(img, line, (10, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, S_SMALL,
                    (0, 165, 255), T_THIN, cv2.LINE_AA)
                y0 += LINE_HS

        if warn_lines:
            y0 += 4
            for line in warn_lines:
                cv2.putText(img, line, (10, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, S_SMALL,
                    (0, 100, 255), T_BOLD, cv2.LINE_AA)
                y0 += LINE_HS

        cv2.putText(img,
            "L=test  R=wis  SPACE=lock/unlock  C=clear hist  "
            "S=save  D=load  X=SAM3 pick  ESC=stop",
            (10, COLOR_H - int(12 * UI_SCALE)),
            cv2.FONT_HERSHEY_SIMPLEX, S_FOOT,
            (200, 200, 200), T_THIN, cv2.LINE_AA)

        cv2.imshow("World Calibration", img)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            break
        elif key == 32:  # SPACE
            if calibration_locked:
                calibration_locked = False
                T_world_cam_locked = None
                print("Calibration UNLOCKED")
            elif T_world_cam is not None and fit_ok and z_ok:
                T_world_cam_locked = T_world_cam.copy()
                calibration_locked = True
                print(f"Calibration LOCKED (residual={residual_m*1000:.1f} mm)")
            else:
                print("Kan niet locken: calibratie nog niet stabiel.")
        elif key in (ord('c'), ord('C')):
            for mid in marker_history:
                marker_history[mid].clear()
            print("History cleared.")
        elif key in (ord('s'), ord('S')):
            T_to_save = T_world_cam_locked if calibration_locked else T_world_cam
            if T_to_save is not None:
                save_calibration(T_to_save, residual_m)
            else:
                print("Geen calibratie om op te slaan.")
        elif key in (ord('d'), ord('D')):
            T_loaded = load_calibration()
            if T_loaded is not None:
                T_world_cam_locked = T_loaded
                calibration_locked = True
        elif key in (ord('x'), ord('X')):
            # Grab the latest raw color frame (without overlays) for SAM3
            run_sam3_pick(latest_color_image.copy(), latest_depth_image)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
