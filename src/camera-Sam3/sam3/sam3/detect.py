#!/usr/bin/python3
"""
pick_planner.py – SAM3-powered robotic pick-point selector
═══════════════════════════════════════════════════════════
Improvements over v1
────────────────────
• Largest-inscribed-circle pick point  (replaces centroid)
• Mask-boundary isolation              (replaces centroid-distance)
• Oriented bounding box + gripper angle (PCA on mask pixels)
• Compactness / surface-flatness metric
• Border-proximity penalty
• Normalized (0–1) pick-point output
• Depth image support (optional)
• Model is loaded once and cached between calls via module-level singleton
• All per-instance metrics printed in a tidy table
• Orientation filter: only selects flat/upside-down parts (flange facing up)
"""

import sys
import time
import torch
import numpy as np
from scipy.ndimage import distance_transform_edt
from PIL import Image, ImageDraw, ImageFont
from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor

# ── CONFIG ────────────────────────────────────────────────────────────────────
IMAGE_PATH      = "object1.png"
DEPTH_PATH      = None          # Optional: path to a single-channel depth PNG/EXR
PROMPT          = "blue objects"
SCORE_THRESHOLD = 0.5

# Scoring weights  (must sum to 1.0)
W_CONFIDENCE  = 0.20   # SAM3 detection confidence
W_ISOLATION   = 0.35   # Mask-boundary clearance from neighbours
W_MASK_AREA   = 0.10   # Larger area = less occluded
W_ACCESSIBLE  = 0.10   # Higher in image = closer to robot arm
W_COMPACTNESS = 0.10   # Circular/flat parts are more reliable to grip
W_BORDER      = 0.15   # Penalise parts near image edge (partially visible)

# ── Orientation filter (flat/upside-down parts only) ──────────────────────────
# A part lying flat (flange up) looks circular from above → high compactness
# and has a roughly square bounding box → aspect ratio close to 1.0.
# Parts standing upright or on their side fail one or both of these checks.
#
# Tune these after reviewing the rejection log on your test images:
MIN_COMPACTNESS = 0.55   # Below this → part is standing upright, skip it
MAX_BBOX_ASPECT = 1.35   # max(w/h, h/w) above this → part is tilted/on its side
# ─────────────────────────────────────────────────────────────────────────────

# Visualisation
DRAW_MASK_OVERLAY = True   # Tint each mask with a semi-transparent colour
# ─────────────────────────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════════════════════════
# Model singleton – load once, reuse for every call
# ══════════════════════════════════════════════════════════════════════════════
_model     = None
_processor = None


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

def mask_to_bool(mask: torch.Tensor) -> np.ndarray:
    m = mask.cpu().numpy()
    if m.ndim == 3:
        m = m.squeeze(0)
    return m.astype(bool)


def largest_inscribed_circle(bool_mask: np.ndarray):
    """
    Return (cx, cy, radius) of the largest circle that fits inside the mask
    using the Euclidean distance transform.  This is the safest pick point:
    it maximises clearance from every mask boundary.
    """
    dt = distance_transform_edt(bool_mask)
    flat_idx = dt.argmax()
    cy, cx   = np.unravel_index(flat_idx, dt.shape)
    radius   = dt[cy, cx]
    return int(cx), int(cy), float(radius)


def oriented_bbox_angle(bool_mask: np.ndarray):
    """
    PCA on mask pixel coordinates → principal axis angle in degrees.
    Returns angle in [-90, 90] measured from the horizontal (image x-axis).
    This is the angle the gripper should rotate to align with the part.
    """
    ys, xs = np.where(bool_mask)
    if len(xs) < 5:
        return 0.0
    pts = np.column_stack([xs - xs.mean(), ys - ys.mean()]).astype(float)
    cov = pts.T @ pts / len(pts)
    _, vecs = np.linalg.eigh(cov)          # vecs[:,1] = principal eigenvector
    principal = vecs[:, 1]                 # largest eigenvalue
    angle_rad = np.arctan2(principal[1], principal[0])
    return float(np.degrees(angle_rad))


def mask_boundary_distances(bool_masks: list[np.ndarray]) -> np.ndarray:
    """
    For each mask i, compute the minimum pixel distance between its boundary
    and the boundary of every other mask j, then take the minimum over all j.

    Returns shape (N,) array of min boundary distances.
    """
    N = len(bool_masks)
    # Pre-compute distance transforms (distance from each pixel to mask i boundary)
    # Boundary = mask XOR eroded mask
    from scipy.ndimage import binary_erosion
    boundaries = []
    for m in bool_masks:
        eroded = binary_erosion(m, iterations=1)
        boundaries.append((m ^ eroded).astype(np.uint8))

    dist_transforms = [distance_transform_edt(1 - b) for b in boundaries]

    isolation = np.zeros(N)
    for i in range(N):
        min_dist = np.inf
        for j in range(N):
            if i == j:
                continue
            # Minimum distance from mask-i boundary pixels to mask-j boundary
            # = min of dt_j evaluated at mask-i boundary pixels
            bnd_i_pixels = boundaries[i].astype(bool)
            if bnd_i_pixels.any():
                dists = dist_transforms[j][bnd_i_pixels]
                min_dist = min(min_dist, dists.min())
        isolation[i] = min_dist if np.isfinite(min_dist) else 0.0
    return isolation


def compactness_score(bool_mask: np.ndarray) -> float:
    """
    Isoperimetric ratio: 4π·Area / Perimeter².
    Circle = 1.0 (perfect), elongated/irregular shapes < 1.0.
    A flat part viewed from above is nearly circular → score close to 1.0.
    A part standing on its side is elongated → score well below 1.0.
    """
    from scipy.ndimage import binary_erosion
    eroded    = binary_erosion(bool_mask, iterations=1)
    boundary  = bool_mask ^ eroded
    perimeter = boundary.sum()
    area      = bool_mask.sum()
    if perimeter == 0:
        return 0.0
    return float(4 * np.pi * area / (perimeter ** 2))


def border_clearance(bool_mask: np.ndarray, img_h: int, img_w: int) -> float:
    """
    Fraction of the mask that is NOT within 5 % of any image border.
    1.0 = fully interior, 0.0 = entirely in the border strip.
    """
    margin_y = int(0.05 * img_h)
    margin_x = int(0.05 * img_w)
    interior = np.zeros_like(bool_mask, dtype=bool)
    interior[margin_y:img_h-margin_y, margin_x:img_w-margin_x] = True
    inside_pixels = (bool_mask & interior).sum()
    total_pixels  = bool_mask.sum()
    return float(inside_pixels / total_pixels) if total_pixels > 0 else 0.0


def bbox_aspect_ratio(box: torch.Tensor) -> float:
    """
    Returns max(w/h, h/w) so the result is always >= 1.0.
    A flat part viewed from above → close to 1.0.
    A part standing upright or on its side → noticeably > 1.0.
    """
    x0, y0, x1, y1 = box.tolist()
    w = max(x1 - x0, 1)
    h = max(y1 - y0, 1)
    ratio = w / h
    return float(max(ratio, 1.0 / ratio))


def depth_at_point(depth_img: np.ndarray | None, cx: int, cy: int,
                   radius: float = 5) -> float | None:
    """
    Return median depth value in a small region around (cx, cy).
    Returns None if depth_img is None.
    """
    if depth_img is None:
        return None
    h, w = depth_img.shape[:2]
    r = max(1, int(radius))
    y0, y1 = max(0, cy-r), min(h, cy+r+1)
    x0, x1 = max(0, cx-r), min(w, cx+r+1)
    patch = depth_img[y0:y1, x0:x1]
    return float(np.median(patch)) if patch.size > 0 else None


# ══════════════════════════════════════════════════════════════════════════════
# Orientation filter
# ══════════════════════════════════════════════════════════════════════════════

def filter_flat_parts(masks, boxes, scores,
                      min_compactness=MIN_COMPACTNESS,
                      max_aspect=MAX_BBOX_ASPECT):
    """
    Keep only instances that appear flat/upside-down when viewed from above.

    Criteria
    --------
    1. Compactness >= min_compactness  →  mask is roughly circular (not elongated)
    2. Bounding-box aspect ratio <= max_aspect  →  footprint is roughly square

    A part lying flat with its flange facing up satisfies both.
    A part standing upright or tilted on its side fails at least one.

    Returns filtered (masks, boxes, scores) tensors plus a list of rejection
    reasons for every discarded instance (useful for threshold tuning).
    """
    keep    = []
    reasons = {}   # idx → rejection reason string

    for i in range(len(scores)):
        bool_m = mask_to_bool(masks[i])
        cmp    = compactness_score(bool_m)
        aspect = bbox_aspect_ratio(boxes[i])

        failed = []
        if cmp < min_compactness:
            failed.append(f"compactness={cmp:.3f} < {min_compactness}")
        if aspect > max_aspect:
            failed.append(f"aspect={aspect:.2f} > {max_aspect}")

        if failed:
            reasons[i] = ", ".join(failed)
        else:
            keep.append(i)

    # Print rejection summary
    if reasons:
        print(f"\n  Orientation filter – rejected {len(reasons)} instance(s):")
        for idx, reason in reasons.items():
            print(f"    Instance #{idx+1}: {reason}")

    if not keep:
        return None, None, None, reasons

    idx_t = torch.tensor(keep)
    return masks[idx_t], boxes[idx_t], scores[idx_t], reasons


# ══════════════════════════════════════════════════════════════════════════════
# Scoring
# ══════════════════════════════════════════════════════════════════════════════

def compute_pick_scores(masks, boxes, scores, img_h, img_w, depth_img=None):
    """
    Compute composite pick scores and per-instance geometry.

    Returns
    -------
    pick_scores : np.ndarray (N,)
    pick_points : list of (cx, cy, radius) – largest inscribed circle
    angles      : list of gripper angles (degrees)
    metrics     : dict of normalised per-instance arrays
    raw_metrics : dict of un-normalised values (for display)
    """
    N = len(scores)
    conf = scores.cpu().numpy()

    bool_masks    = [mask_to_bool(masks[i]) for i in range(N)]
    pick_points   = [largest_inscribed_circle(m) for m in bool_masks]
    angles        = [oriented_bbox_angle(m)       for m in bool_masks]
    areas         = np.array([m.sum()             for m in bool_masks], dtype=float)

    # Mask-boundary isolation (expensive but accurate)
    print("  Computing mask-boundary isolation…")
    isolation_raw = mask_boundary_distances(bool_masks)

    # Compactness
    compactness_raw = np.array([compactness_score(m) for m in bool_masks])

    # Border clearance
    border_raw = np.array([border_clearance(m, img_h, img_w) for m in bool_masks])

    # Accessibility: prefer high in image (low cy)
    cy_vals      = np.array([p[1] for p in pick_points], dtype=float)
    accessible_r = img_h - cy_vals

    def norm(arr):
        rng = arr.max() - arr.min()
        return (arr - arr.min()) / rng if rng > 0 else np.ones_like(arr)

    conf_n        = norm(conf)
    isolation_n   = norm(isolation_raw)
    mask_area_n   = norm(areas)
    accessible_n  = norm(accessible_r)
    compactness_n = norm(compactness_raw)
    border_n      = norm(border_raw)

    pick_scores = (
        W_CONFIDENCE  * conf_n        +
        W_ISOLATION   * isolation_n   +
        W_MASK_AREA   * mask_area_n   +
        W_ACCESSIBLE  * accessible_n  +
        W_COMPACTNESS * compactness_n +
        W_BORDER      * border_n
    )

    metrics = {
        "confidence"  : conf_n,
        "isolation"   : isolation_n,
        "mask_area"   : mask_area_n,
        "accessible"  : accessible_n,
        "compactness" : compactness_n,
        "border"      : border_n,
    }
    raw_metrics = {
        "confidence"  : conf,
        "isolation_px": isolation_raw,
        "area_px"     : areas,
        "cy"          : cy_vals,
        "compactness" : compactness_raw,
        "border_frac" : border_raw,
        "angle_deg"   : np.array(angles),
    }

    # Attach depth values if available
    if depth_img is not None:
        depths = np.array([depth_at_point(depth_img, p[0], p[1]) for p in pick_points],
                          dtype=float)
        raw_metrics["depth"] = depths

    return pick_scores, pick_points, angles, metrics, raw_metrics


# ══════════════════════════════════════════════════════════════════════════════
# Visualisation
# ══════════════════════════════════════════════════════════════════════════════

PALETTE = [
    (255, 80,  80,  90),   # red
    (80,  200, 120, 90),   # green
    (80,  140, 255, 90),   # blue
    (255, 200,  50, 90),   # yellow
    (200,  80, 255, 90),   # purple
    (50,  220, 220, 90),   # cyan
]


def draw_results(image: Image.Image, masks, boxes, scores,
                 pick_scores, pick_points, angles, metrics, best_idx):
    # ── mask overlays ──────────────────────────────────────────────────────
    if DRAW_MASK_OVERLAY:
        overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
        for i, m in enumerate(masks):
            colour = PALETTE[i % len(PALETTE)]
            bool_m = mask_to_bool(m)
            layer  = np.zeros((*bool_m.shape, 4), dtype=np.uint8)
            layer[bool_m] = colour
            overlay = Image.alpha_composite(
                overlay, Image.fromarray(layer, "RGBA"))
        image = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")

    draw = ImageDraw.Draw(image)
    N    = len(scores)

    for i in range(N):
        cx, cy, r_ic = pick_points[i]
        x0, y0, x1, y1 = [int(v) for v in boxes[i].tolist()]
        is_best = (i == best_idx)

        # Bounding box
        box_color = "lime" if is_best else "#aaaaaa"
        box_width = 4      if is_best else 1
        draw.rectangle([x0, y0, x1, y1], outline=box_color, width=box_width)

        # Inscribed circle
        ic_color = "lime" if is_best else "#aaaaaa"
        r_ic_i   = int(r_ic)
        draw.ellipse([cx-r_ic_i, cy-r_ic_i, cx+r_ic_i, cy+r_ic_i],
                     outline=ic_color, width=2)

        # Centroid dot
        dot = 4
        draw.ellipse([cx-dot, cy-dot, cx+dot, cy+dot], fill="yellow")

        # Gripper angle indicator
        angle_rad = np.radians(angles[i])
        arm_len   = r_ic_i + 12
        ex = cx + arm_len * np.cos(angle_rad)
        ey = cy + arm_len * np.sin(angle_rad)
        draw.line([cx, cy, int(ex), int(ey)], fill="cyan", width=2)
        draw.line([cx, cy,
                   int(cx - arm_len * np.cos(angle_rad)),
                   int(cy - arm_len * np.sin(angle_rad))],
                  fill="cyan", width=2)

        # Score label
        label = (
            f"#{i+1} pick={pick_scores[i]:.2f} "
            f"[c={metrics['confidence'][i]:.2f} "
            f"iso={metrics['isolation'][i]:.2f} "
            f"cmp={metrics['compactness'][i]:.2f} "
            f"brd={metrics['border'][i]:.2f}]"
        )
        draw.text((x0, max(0, y0 - 16)), label, fill=box_color)

    # ── Best-pick crosshair ────────────────────────────────────────────────
    cx, cy, _ = pick_points[best_idx]
    r = 20
    draw.ellipse([cx-r, cy-r, cx+r, cy+r], outline="red", width=3)
    draw.line([cx - r*2, cy, cx + r*2, cy], fill="red", width=2)
    draw.line([cx, cy - r*2, cx, cy + r*2], fill="red", width=2)
    draw.text((cx + r + 4, cy - 8), "PICK", fill="red")

    return image


def print_table(N, pick_scores, pick_points, angles, raw_metrics):
    """Pretty-print a summary table to stdout."""
    header = (f"{'#':>3}  {'pick':>5}  {'conf':>5}  {'iso(px)':>7}  "
              f"{'area':>7}  {'cmpct':>5}  {'border':>6}  "
              f"{'cx':>5}  {'cy':>5}  {'angle':>6}  {'depth':>7}")
    print("\n" + "─" * len(header))
    print(header)
    print("─" * len(header))
    ranked = sorted(range(N), key=lambda i: pick_scores[i], reverse=True)
    for rank, idx in enumerate(ranked):
        cx, cy, _ = pick_points[idx]
        depth_str = (f"{raw_metrics['depth'][idx]:>7.1f}"
                     if "depth" in raw_metrics else "    n/a")
        marker = " ◀ BEST" if rank == 0 else ""
        print(
            f"{idx+1:>3}  "
            f"{pick_scores[idx]:>5.3f}  "
            f"{raw_metrics['confidence'][idx]:>5.3f}  "
            f"{raw_metrics['isolation_px'][idx]:>7.1f}  "
            f"{raw_metrics['area_px'][idx]:>7.0f}  "
            f"{raw_metrics['compactness'][idx]:>5.3f}  "
            f"{raw_metrics['border_frac'][idx]:>6.3f}  "
            f"{cx:>5}  {cy:>5}  "
            f"{raw_metrics['angle_deg'][idx]:>6.1f}°  "
            f"{depth_str}"
            f"{marker}"
        )
    print("─" * len(header))


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def run(image_path=IMAGE_PATH, depth_path=DEPTH_PATH,
        prompt=PROMPT, score_threshold=SCORE_THRESHOLD):

    _, processor = get_model()

    # ── Load images ────────────────────────────────────────────────────────
    image    = Image.open(image_path)
    img_w, img_h = image.size
    print(f"Image: {image_path}  ({img_w}×{img_h}px)")

    depth_img = None
    if depth_path:
        raw = Image.open(depth_path).convert("L")  # single channel
        depth_img = np.array(raw, dtype=float)
        print(f"Depth map loaded: {depth_path}")

    # ── Inference ──────────────────────────────────────────────────────────
    print(f"Running SAM3 with prompt: {prompt}…")
    with torch.autocast(device_type="cuda", dtype=torch.float32):
        state  = processor.set_image(image)
        output = processor.set_text_prompt(state=state, prompt=prompt)

    masks  = output["masks"]
    boxes  = output["boxes"]
    scores = output["scores"]

    # ── Confidence threshold ───────────────────────────────────────────────
    keep   = scores >= score_threshold
    masks  = masks[keep];  boxes = boxes[keep];  scores = scores[keep]

    if len(scores) == 0:
        print("No objects detected. Lower SCORE_THRESHOLD or change PROMPT.")
        return None

    print(f"Detected {len(scores)} instance(s) above confidence threshold {score_threshold}.")

    # ── Orientation filter: keep only flat (upside-down) parts ────────────
    # Parts lying flat with their flange up look nearly circular from above
    # (high compactness) and have a roughly square bounding box (aspect ≈ 1).
    # Parts standing upright or tilted fail one or both checks and are skipped.
    print(f"\nApplying orientation filter "
          f"(min_compactness={MIN_COMPACTNESS}, max_aspect={MAX_BBOX_ASPECT})…")

    masks, boxes, scores, rejections = filter_flat_parts(
        masks, boxes, scores,
        min_compactness=MIN_COMPACTNESS,
        max_aspect=MAX_BBOX_ASPECT,
    )

    if masks is None:
        print(
            "\nNo flat/upside-down parts passed the orientation filter.\n"
            "Tips:\n"
            "  • Lower MIN_COMPACTNESS  (currently {MIN_COMPACTNESS}) if valid parts are being rejected\n"
            "  • Raise MAX_BBOX_ASPECT  (currently {MAX_BBOX_ASPECT}) if valid parts are being rejected\n"
            "  • Check the rejection log above to see which criterion failed"
        )
        return None

    print(f"  {len(scores)} flat instance(s) passed the orientation filter.")
    # ──────────────────────────────────────────────────────────────────────

    # ── Score ──────────────────────────────────────────────────────────────
    pick_scores, pick_points, angles, metrics, raw_metrics = compute_pick_scores(
        masks, boxes, scores, img_h, img_w, depth_img)

    best_idx     = int(pick_scores.argmax())
    cx, cy, r_ic = pick_points[best_idx]

    # Normalised coordinates (resolution-independent, ready for robot controller)
    cx_norm = cx / img_w
    cy_norm = cy / img_h

    # ── Report ─────────────────────────────────────────────────────────────
    print_table(len(scores), pick_scores, pick_points, angles, raw_metrics)

    print(f"\n{'═'*42}")
    print(f"  BEST PICK POINT")
    print(f"{'─'*42}")
    print(f"  Instance          : #{best_idx + 1}")
    print(f"  Pixel position    : ({cx}, {cy})")
    print(f"  Normalised (0–1)  : ({cx_norm:.4f}, {cy_norm:.4f})")
    print(f"  Inscribed radius  : {r_ic:.1f} px   ← gripper clearance")
    print(f"  Gripper angle     : {angles[best_idx]:.1f}°")
    print(f"  Pick score        : {pick_scores[best_idx]:.3f}")
    if "depth" in raw_metrics:
        print(f"  Depth value       : {raw_metrics['depth'][best_idx]:.1f}")
    print(f"{'═'*42}\n")

    # ── Draw & save ────────────────────────────────────────────────────────
    annotated = draw_results(
        image.copy(), masks, boxes, scores,
        pick_scores, pick_points, angles, metrics, best_idx)
    out_path = "output_pick.jpg"
    annotated.save(out_path)
    print(f"Annotated image saved → {out_path}")

    return {
        "best_instance"   : best_idx + 1,
        "pixel"           : (cx, cy),
        "normalised"      : (cx_norm, cy_norm),
        "inscribed_radius": r_ic,
        "gripper_angle"   : angles[best_idx],
        "pick_score"      : float(pick_scores[best_idx]),
        "depth"           : raw_metrics.get("depth", [None] * (best_idx+1))[best_idx],
        "all_scores"      : pick_scores.tolist(),
        "all_points"      : [(p[0], p[1]) for p in pick_points],
        "all_angles"      : angles,
    }


if __name__ == "__main__":
    result = run()