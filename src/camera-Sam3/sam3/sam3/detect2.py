#!/usr/bin/python3
"""
pick_planner.py – SAM3-powered robotic pick-point selector  (v3)
═══════════════════════════════════════════════════════════════
What changed in v3
──────────────────
Assumption: parts are NOT stacked, but they may touch.  The gripper only
fails when its fingers / body collide with a neighbour, so scoring is now
driven entirely by FREE SPACE around the candidate, not by mask-to-mask
boundary distance.

Key new ideas
─────────────
• Global free-space map         (background = NOT any mask) + one distance
                                 transform → O(1) clearance look-up anywhere.
• Free-perimeter ratio          fraction of the object's outline that faces
                                 empty space (vs. touching a neighbour).
                                 This is the single strongest signal that a
                                 part can be picked without collision.
• Gripper-finger clearance      free-space distance at the two points where
                                 the jaws will actually close, sampled along
                                 the minor axis at half-width + finger pad.
• Corner clearance              free-space distance at the 4 oriented-bbox
                                 corners — catches collisions the finger
                                 sample misses (e.g. wide gripper bodies).
• Area-ratio sanity             rejects masks that are >1.8× or <0.5× the
                                 median area (touching pair merged into one
                                 detection, or a partial sliver).
• O(N) cost                     v2 ran an O(N²) loop of distance transforms;
                                 v3 does ONE distance transform total.
"""

import sys
import time
import torch
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_erosion, binary_dilation
from PIL import Image, ImageDraw
from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor

# ── CONFIG ────────────────────────────────────────────────────────────────────
IMAGE_PATH      = "object2.png"
DEPTH_PATH      = None
PROMPT          = "black objects"
SCORE_THRESHOLD = 0.5

# ── Gripper geometry (pixels) ────────────────────────────────────────────────
# Calibrate these once for your camera + gripper.  They drive the finger /
# corner clearance checks and the "is this enough room?" decision.
GRIPPER_FINGER_PAD   = 8    # extra px each finger needs beyond the part edge
GRIPPER_BODY_RADIUS  = 14   # radius of the gripper body above the part
MIN_FINGER_CLEARANCE = 6    # px of free space required at each finger point

# ── Scoring weights (must sum to 1.0) ────────────────────────────────────────
W_FREE_PERIM   = 0.40   # fraction of outline facing empty space
W_FINGER_CLR   = 0.25   # free space where the jaws will close
W_CORNER_CLR   = 0.15   # free space at the 4 oriented-bbox corners
W_CONFIDENCE   = 0.05   # SAM3 detection confidence
W_BORDER       = 0.10   # not clipped by image edge
W_AREA_SANITY  = 0.05   # close to median area = single clean instance

# ── Area-ratio sanity filter ─────────────────────────────────────────────────
AREA_RATIO_MIN = 0.50   # mask < 0.5× median area → likely partial / occluded
AREA_RATIO_MAX = 1.80   # mask > 1.8× median area → likely two touching parts

# ── Optional orientation filter (kept from v2, looser defaults) ──────────────
USE_ORIENTATION_FILTER = False   # off by default; parts can touch & rotate freely
MIN_COMPACTNESS = 0.45
MAX_BBOX_ASPECT = 1.60

DRAW_MASK_OVERLAY = True
# ─────────────────────────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════════════════════════
# Model singleton
# ══════════════════════════════════════════════════════════════════════════════
_model = _processor = None
def get_model():
    global _model, _processor
    if _model is None:
        print("Loading SAM3 model (first call only)…")
        t0 = time.time()
        _model     = build_sam3_image_model().float()
        _processor = Sam3Processor(_model)
        print(f"Model ready in {time.time()-t0:.1f}s")
    return _model, _processor


# ══════════════════════════════════════════════════════════════════════════════
# Geometry helpers
# ══════════════════════════════════════════════════════════════════════════════

def mask_to_bool(mask):
    m = mask.cpu().numpy()
    if m.ndim == 3:
        m = m.squeeze(0)
    return m.astype(bool)


def largest_inscribed_circle(bool_mask):
    """Safest on-part pick: maximises distance to the part's own boundary."""
    dt = distance_transform_edt(bool_mask)
    cy, cx = np.unravel_index(dt.argmax(), dt.shape)
    return int(cx), int(cy), float(dt[cy, cx])


def principal_axes(bool_mask):
    """
    Return (angle_deg, major_axis_vec, minor_axis_vec) via PCA.
    angle_deg = orientation of the MAJOR axis from the image x-axis.
    """
    ys, xs = np.where(bool_mask)
    if len(xs) < 5:
        return 0.0, np.array([1., 0.]), np.array([0., 1.])
    pts = np.column_stack([xs - xs.mean(), ys - ys.mean()]).astype(float)
    cov = pts.T @ pts / len(pts)
    vals, vecs = np.linalg.eigh(cov)
    major = vecs[:, 1]                       # largest eigenvalue
    minor = np.array([-major[1], major[0]])  # 90° rotation
    angle = float(np.degrees(np.arctan2(major[1], major[0])))
    return angle, major, minor


def oriented_bbox(bool_mask, major, minor):
    """
    Project all mask pixels onto (major, minor) → axis-aligned extents in that
    frame → 4 oriented-bbox corners back in image coords + half-widths.
    """
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
            p = centre + sa * hw_major * major + sb * hw_minor * minor
            corners.append(p)
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
    """Free-space distance at (x, y), clamped to image bounds."""
    h, w = free_dt.shape
    x = int(np.clip(round(x), 0, w - 1))
    y = int(np.clip(round(y), 0, h - 1))
    return float(free_dt[y, x])


# ══════════════════════════════════════════════════════════════════════════════
# Scene-level free-space analysis (the heart of v3)
# ══════════════════════════════════════════════════════════════════════════════

def build_freespace(bool_masks, img_h, img_w):
    """
    Free-space mask = pixels that belong to NO object.
    Returns the free-space distance transform: at every pixel, the distance to
    the nearest occupied pixel (any mask).  One DT call instead of N².
    """
    occupied = np.zeros((img_h, img_w), dtype=bool)
    for m in bool_masks:
        occupied |= m
    free = ~occupied
    free_dt = distance_transform_edt(free)
    return occupied, free_dt


def free_perimeter_ratio(bool_mask, occupied, dilate_iter=2):
    """
    Of the mask's outline, what fraction borders FREE space (vs. another mask)?

    Method: dilate the mask by a few pixels.  The 'ring' = dilation - mask.
    Pixels in the ring that are NOT occupied by another mask → free outline.
    1.0 = fully surrounded by air (ideal), 0.0 = fully wedged between parts.
    """
    dilated = binary_dilation(bool_mask, iterations=dilate_iter)
    ring    = dilated & ~bool_mask
    if ring.sum() == 0:
        return 0.0
    # 'occupied' includes THIS mask too, so subtract it
    others_occ = occupied & ~bool_mask
    free_ring  = ring & ~others_occ
    return float(free_ring.sum() / ring.sum())


def gripper_finger_clearance(free_dt, centre, minor, hw_minor):
    """
    Sample free-space distance at the two finger landing points: along the
    MINOR axis (perpendicular to the part's length), just outside the part.
    Returns (min_of_two, mean_of_two) — min is what matters for collision.
    """
    reach = hw_minor + GRIPPER_FINGER_PAD
    p1 = centre + reach * minor
    p2 = centre - reach * minor
    c1 = sample_clearance(free_dt, p1[0], p1[1])
    c2 = sample_clearance(free_dt, p2[0], p2[1])
    return min(c1, c2), 0.5 * (c1 + c2), (p1, p2)


def corner_clearance(free_dt, corners):
    """Min free-space distance over the 4 oriented-bbox corners."""
    vals = [sample_clearance(free_dt, c[0], c[1]) for c in corners]
    return float(min(vals)), vals


# ══════════════════════════════════════════════════════════════════════════════
# Filters
# ══════════════════════════════════════════════════════════════════════════════

def filter_orientation(masks, boxes, scores):
    keep, reasons = [], {}
    for i in range(len(scores)):
        bm = mask_to_bool(masks[i])
        c  = compactness_score(bm)
        a  = bbox_aspect_ratio(boxes[i])
        fail = []
        if c < MIN_COMPACTNESS: fail.append(f"compactness={c:.2f}")
        if a > MAX_BBOX_ASPECT: fail.append(f"aspect={a:.2f}")
        (reasons.__setitem__(i, ", ".join(fail)) if fail else keep.append(i))
    if reasons:
        print(f"\n  Orientation filter rejected {len(reasons)}:")
        for i, r in reasons.items():
            print(f"    #{i+1}: {r}")
    if not keep:
        return None, None, None
    idx = torch.tensor(keep)
    return masks[idx], boxes[idx], scores[idx]


def filter_area_sanity(masks, boxes, scores):
    """Drop masks whose area is far from the median (probably merged or partial)."""
    areas = np.array([mask_to_bool(m).sum() for m in masks], dtype=float)
    if len(areas) < 2:
        return masks, boxes, scores, areas
    median = np.median(areas)
    keep, reasons = [], {}
    for i, a in enumerate(areas):
        r = a / median
        if r < AREA_RATIO_MIN:
            reasons[i] = f"area={a:.0f} ({r:.2f}× median) — too small / partial"
        elif r > AREA_RATIO_MAX:
            reasons[i] = f"area={a:.0f} ({r:.2f}× median) — likely merged pair"
        else:
            keep.append(i)
    if reasons:
        print(f"\n  Area-sanity filter rejected {len(reasons)} "
              f"(median area = {median:.0f}px):")
        for i, r in reasons.items():
            print(f"    #{i+1}: {r}")
    if not keep:
        return None, None, None, areas
    idx = torch.tensor(keep)
    return masks[idx], boxes[idx], scores[idx], areas[keep]


# ══════════════════════════════════════════════════════════════════════════════
# Scoring
# ══════════════════════════════════════════════════════════════════════════════

def compute_pick_scores(masks, boxes, scores, img_h, img_w, depth_img=None):
    N = len(scores)
    conf = scores.cpu().numpy()
    bool_masks = [mask_to_bool(masks[i]) for i in range(N)]

    # ── ONE distance transform for the whole scene ────────────────────────
    print("  Building free-space map…")
    occupied, free_dt = build_freespace(bool_masks, img_h, img_w)

    pick_points, angles  = [], []
    free_perim, fing_min, fing_mean, corner_min = [], [], [], []
    finger_pts_all, corner_pts_all = [], []
    areas = []

    for i, bm in enumerate(bool_masks):
        cx, cy, r_ic = largest_inscribed_circle(bm)
        pick_points.append((cx, cy, r_ic))

        ang, major, minor = principal_axes(bm)
        angles.append(ang)
        centre, hw_maj, hw_min, corners = oriented_bbox(bm, major, minor)
        corner_pts_all.append(corners)

        fmin, fmean, fpts = gripper_finger_clearance(free_dt, centre,
                                                     minor, hw_min)
        fing_min.append(fmin); fing_mean.append(fmean); finger_pts_all.append(fpts)

        cmin, _ = corner_clearance(free_dt, corners)
        corner_min.append(cmin)

        free_perim.append(free_perimeter_ratio(bm, occupied))
        areas.append(bm.sum())

    free_perim  = np.array(free_perim)
    fing_min    = np.array(fing_min)
    fing_mean   = np.array(fing_mean)
    corner_min  = np.array(corner_min)
    areas       = np.array(areas, dtype=float)

    # Area sanity score: 1.0 at median, falling off linearly
    median_area = np.median(areas) if len(areas) else 1.0
    area_dev    = np.abs(areas - median_area) / max(median_area, 1.0)
    area_sanity = np.clip(1.0 - area_dev, 0.0, 1.0)

    border_raw = np.array([border_clearance(bm, img_h, img_w)
                           for bm in bool_masks])

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

    # Hard veto: if min finger clearance < threshold, score → 0
    vetoed = fing_min < MIN_FINGER_CLEARANCE
    if vetoed.any():
        print(f"\n  Veto: {vetoed.sum()} instance(s) below "
              f"MIN_FINGER_CLEARANCE={MIN_FINGER_CLEARANCE}px")
        pick_scores = np.where(vetoed, 0.0, pick_scores)

    raw = {
        "confidence" : conf,
        "free_perim" : free_perim,
        "finger_min" : fing_min,
        "finger_mean": fing_mean,
        "corner_min" : corner_min,
        "area"       : areas,
        "border"     : border_raw,
        "angle"      : np.array(angles),
        "vetoed"     : vetoed,
    }
    if depth_img is not None:
        raw["depth"] = np.array([
            float(np.median(depth_img[
                max(0, p[1]-3):p[1]+4, max(0, p[0]-3):p[0]+4]))
            for p in pick_points])

    return (pick_scores, pick_points, angles, raw,
            finger_pts_all, corner_pts_all, free_dt)


# ══════════════════════════════════════════════════════════════════════════════
# Visualisation
# ══════════════════════════════════════════════════════════════════════════════
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
            overlay = Image.alpha_composite(overlay,
                        Image.fromarray(layer, "RGBA"))
        image = Image.alpha_composite(image.convert("RGBA"),
                                      overlay).convert("RGB")
    draw = ImageDraw.Draw(image)

    for i in range(len(pick_scores)):
        cx, cy, r_ic = pick_points[i]
        x0, y0, x1, y1 = [int(v) for v in boxes[i].tolist()]
        is_best = (i == best_idx)
        vetoed  = bool(raw["vetoed"][i])

        col = "lime" if is_best else ("#ff5050" if vetoed else "#aaaaaa")
        w   = 4 if is_best else 1
        draw.rectangle([x0,y0,x1,y1], outline=col, width=w)

        # Inscribed pick circle
        ri = int(r_ic)
        draw.ellipse([cx-ri, cy-ri, cx+ri, cy+ri], outline=col, width=2)
        draw.ellipse([cx-3, cy-3, cx+3, cy+3], fill="yellow")

        # Gripper jaw axis (cyan line through the part)
        ang = np.radians(angles[i])
        L   = ri + 16
        ex, ey = cx + L*np.cos(ang), cy + L*np.sin(ang)
        draw.line([cx, cy, int(ex), int(ey)], fill="cyan", width=2)
        draw.line([cx, cy, int(2*cx-ex), int(2*cy-ey)], fill="cyan", width=2)

        # Finger landing points (orange = OK, red = veto)
        p1, p2 = finger_pts_all[i]
        fc = "red" if vetoed else "orange"
        for p in (p1, p2):
            px, py = int(p[0]), int(p[1])
            draw.ellipse([px-5, py-5, px+5, py+5], outline=fc, width=2)

        # Corner clearance dots (small magenta)
        for c in corner_pts_all[i]:
            px, py = int(c[0]), int(c[1])
            draw.ellipse([px-2, py-2, px+2, py+2], fill="magenta")

        label = (f"#{i+1} {pick_scores[i]:.2f} "
                 f"fp={raw['free_perim'][i]:.2f} "
                 f"fc={raw['finger_min'][i]:.0f}px "
                 f"cc={raw['corner_min'][i]:.0f}px")
        draw.text((x0, max(0, y0-14)), label, fill=col)

    # Best-pick crosshair
    cx, cy, _ = pick_points[best_idx]
    r = 22
    draw.ellipse([cx-r, cy-r, cx+r, cy+r], outline="red", width=3)
    draw.line([cx-2*r, cy, cx+2*r, cy], fill="red", width=2)
    draw.line([cx, cy-2*r, cx, cy+2*r], fill="red", width=2)
    draw.text((cx+r+4, cy-8), "PICK", fill="red")
    return image


def print_table(pick_scores, pick_points, angles, raw):
    hdr = (f"{'#':>3} {'pick':>5} {'free%':>6} {'fingMin':>7} "
           f"{'fingMean':>8} {'cornMin':>7} {'area':>7} {'conf':>5} "
           f"{'brd':>5} {'ang':>6}  veto")
    print("\n" + "─"*len(hdr)); print(hdr); print("─"*len(hdr))
    order = sorted(range(len(pick_scores)),
                   key=lambda i: pick_scores[i], reverse=True)
    for rank, i in enumerate(order):
        mark = " ◀ BEST" if rank == 0 and pick_scores[i] > 0 else ""
        print(f"{i+1:>3} {pick_scores[i]:>5.3f} "
              f"{raw['free_perim'][i]*100:>5.1f}% "
              f"{raw['finger_min'][i]:>7.1f} "
              f"{raw['finger_mean'][i]:>8.1f} "
              f"{raw['corner_min'][i]:>7.1f} "
              f"{raw['area'][i]:>7.0f} "
              f"{raw['confidence'][i]:>5.3f} "
              f"{raw['border'][i]:>5.2f} "
              f"{raw['angle'][i]:>5.1f}°  "
              f"{'YES' if raw['vetoed'][i] else ' . '}{mark}")
    print("─"*len(hdr))


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def run(image_path=IMAGE_PATH, depth_path=DEPTH_PATH,
        prompt=PROMPT, score_threshold=SCORE_THRESHOLD):
    _, processor = get_model()

    image = Image.open(image_path)
    img_w, img_h = image.size
    print(f"Image: {image_path} ({img_w}×{img_h})")

    depth_img = None
    if depth_path:
        depth_img = np.array(Image.open(depth_path).convert("L"), dtype=float)
        print(f"Depth loaded: {depth_path}")

    print(f"Running SAM3 with prompt: {prompt!r}…")
    with torch.autocast(device_type="cuda", dtype=torch.float32):
        state  = processor.set_image(image)
        output = processor.set_text_prompt(state=state, prompt=prompt)
    masks, boxes, scores = output["masks"], output["boxes"], output["scores"]

    keep = scores >= score_threshold
    masks, boxes, scores = masks[keep], boxes[keep], scores[keep]
    if len(scores) == 0:
        print("No detections above threshold."); return None
    print(f"Detected {len(scores)} instance(s).")

    if USE_ORIENTATION_FILTER:
        print(f"\nOrientation filter "
              f"(min_compactness={MIN_COMPACTNESS}, max_aspect={MAX_BBOX_ASPECT})…")
        masks, boxes, scores = filter_orientation(masks, boxes, scores)
        if masks is None:
            print("Nothing passed orientation filter."); return None

    print(f"\nArea-sanity filter "
          f"(keep [{AREA_RATIO_MIN}, {AREA_RATIO_MAX}]× median area)…")
    masks, boxes, scores, _ = filter_area_sanity(masks, boxes, scores)
    if masks is None:
        print("Nothing passed area filter."); return None
    print(f"  {len(scores)} instance(s) remain.")

    (pick_scores, pick_points, angles, raw,
     finger_pts, corner_pts, _) = compute_pick_scores(
        masks, boxes, scores, img_h, img_w, depth_img)

    if pick_scores.max() == 0:
        print("\nAll candidates vetoed — no safe pick.")
        return None

    best = int(pick_scores.argmax())
    cx, cy, r_ic = pick_points[best]
    print_table(pick_scores, pick_points, angles, raw)

    print(f"\n{'═'*46}")
    print(f"  BEST PICK")
    print(f"{'─'*46}")
    print(f"  Instance         : #{best+1}")
    print(f"  Pixel            : ({cx}, {cy})")
    print(f"  Normalised       : ({cx/img_w:.4f}, {cy/img_h:.4f})")
    print(f"  Inscribed radius : {r_ic:.1f} px")
    print(f"  Gripper angle    : {angles[best]:.1f}°")
    print(f"  Free-perim ratio : {raw['free_perim'][best]:.2%}")
    print(f"  Finger clearance : min={raw['finger_min'][best]:.1f}px  "
          f"mean={raw['finger_mean'][best]:.1f}px")
    print(f"  Corner clearance : {raw['corner_min'][best]:.1f}px")
    print(f"  Pick score       : {pick_scores[best]:.3f}")
    print(f"{'═'*46}\n")

    annotated = draw_results(image.copy(), masks, boxes,
                             pick_scores, pick_points, angles, raw,
                             finger_pts, corner_pts, best)
    annotated.save("output_pick.jpg")
    print("Annotated image → output_pick.jpg")

    return {
        "best_instance"  : best + 1,
        "pixel"          : (cx, cy),
        "normalised"     : (cx/img_w, cy/img_h),
        "inscribed_radius": r_ic,
        "gripper_angle"  : angles[best],
        "pick_score"     : float(pick_scores[best]),
        "free_perimeter" : float(raw["free_perim"][best]),
        "finger_clearance": float(raw["finger_min"][best]),
        "corner_clearance": float(raw["corner_min"][best]),
        "all_scores"     : pick_scores.tolist(),
        "all_points"     : [(p[0], p[1]) for p in pick_points],
        "all_angles"     : angles,
    }


if __name__ == "__main__":
    run()
