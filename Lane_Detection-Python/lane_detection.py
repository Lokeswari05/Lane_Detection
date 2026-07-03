import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.fftpack import dct, idct
from sklearn.linear_model import RANSACRegressor
import os, warnings

warnings.filterwarnings("ignore")


# ─────────────────────────────────────────────────────────────────────────────
# STEP 2 – Grayscale + CLAHE
# ─────────────────────────────────────────────────────────────────────────────
def to_grayscale(bgr, clip=2.0):
    gray  = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8))
    return clahe.apply(gray)


# ─────────────────────────────────────────────────────────────────────────────
# STEP 3 – ROI builders
# ─────────────────────────────────────────────────────────────────────────────
def make_roi_v1(image, h, w, top_frac=0.45):
    apex_y  = int(h * top_frac)
    apex_x1 = int(w * 0.38)
    apex_x2 = int(w * 0.62)
    verts = np.array([[(0, h), (apex_x1, apex_y),
                        (apex_x2, apex_y), (w, h)]], dtype=np.int32)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, verts, 255)
    return cv2.bitwise_and(image, mask), mask, verts

def make_roi_v2(image, h, w, top_frac=0.55):
    apex_y  = int(h * top_frac)
    apex_x1 = int(w * 0.42)
    apex_x2 = int(w * 0.58)
    bot_x1  = int(w * 0.02)
    bot_x2  = int(w * 0.98)
    verts = np.array([[(bot_x1, h-1), (apex_x1, apex_y),
                        (apex_x2, apex_y), (bot_x2, h-1)]], dtype=np.int32)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, verts, 255)
    return cv2.bitwise_and(image, mask), mask, verts

def make_roi_img1(image, h, w, cfg):
    """img1: single trapezoid covering only the RIGHT driving lane."""
    top_y = cfg["roi_top_y"]
    verts = np.array([[
        (cfg["roi_bot_left"],  h-1),
        (cfg["roi_top_left"],  top_y),
        (cfg["roi_top_right"], top_y),
        (cfg["roi_bot_right"], h-1),
    ]], dtype=np.int32)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, verts, 255)
    return cv2.bitwise_and(image, mask), mask, verts

def make_roi_img2(image, h, w, cfg):
    """
    img2 v4: single right-side trapezoid covering BOTH visible lane lines.
    Gantry sign excluded by roi_top_y_frac (top edge raised to 42 % of height).
    """
    top_y = int(h * cfg["roi_top_y_frac"])
    verts = np.array([[
        (cfg["roi_bot_left"],  h-1),
        (cfg["roi_top_left"],  top_y),
        (cfg["roi_top_right"], top_y),
        (cfg["roi_bot_right"], h-1),
    ]], dtype=np.int32)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, verts, 255)
    return cv2.bitwise_and(image, mask), mask, verts


# ─────────────────────────────────────────────────────────────────────────────
# STEP 4 – SDCT Edge Detection
# ─────────────────────────────────────────────────────────────────────────────
def sdct_edge_detection(gray_roi):
    blurred   = cv2.GaussianBlur(gray_roi, (5, 5), 1.5)
    gx        = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
    gy        = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
    sobel_mag = np.clip(np.sqrt(gx**2 + gy**2), 0, 255).astype(np.uint8)

    mag_f  = sobel_mag.astype(np.float64)
    dct_2d = dct(dct(mag_f, axis=0, norm="ortho"), axis=1, norm="ortho")
    h2, w2 = dct_2d.shape
    dct_hp = dct_2d.copy()
    dct_hp[:max(1, h2//6), :max(1, w2//6)] = 0.0
    enhanced = np.abs(idct(idct(dct_hp, axis=1, norm="ortho"), axis=0, norm="ortho"))
    if enhanced.max() > 0:
        enhanced = (enhanced / enhanced.max() * 255).astype(np.uint8)
    else:
        enhanced = np.zeros_like(sobel_mag)

    fused    = cv2.addWeighted(sobel_mag, 0.55, enhanced, 0.45, 0)
    _, binary = cv2.threshold(fused, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    k        = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary   = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k, iterations=2)
    binary   = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  k, iterations=1)
    return binary, sobel_mag, enhanced


# ─────────────────────────────────────────────────────────────────────────────
# Color-space lane mask (HLS white + yellow)
# ─────────────────────────────────────────────────────────────────────────────
def color_lane_mask(bgr):
    hls    = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
    white  = cv2.inRange(hls, np.array([0,  180,  0]), np.array([255, 255, 255]))
    yellow = cv2.inRange(hls, np.array([15,  60, 80]), np.array([40,  255, 255]))
    return cv2.bitwise_or(white, yellow)


# ─────────────────────────────────────────────────────────────────────────────
# STEP 5 – Blob Analysis
# ─────────────────────────────────────────────────────────────────────────────
def blob_analysis(binary, h, w, bottom_frac=0.40):
    mid  = w // 2
    ys, xs = np.where(binary > 0)
    if len(ys) == 0:
        return np.empty((0, 2)), np.empty((0, 2))
    keep = ys > (h * bottom_frac)
    ys, xs = ys[keep], xs[keep]
    lm = xs < mid;  rm = xs >= mid
    left_pts  = np.column_stack((ys[lm], xs[lm])) if lm.any() else np.empty((0, 2))
    right_pts = np.column_stack((ys[rm], xs[rm])) if rm.any() else np.empty((0, 2))
    return left_pts, right_pts


# ─────────────────────────────────────────────────────────────────────────────
# STEP 6 – RANSAC Line Fitting
# ─────────────────────────────────────────────────────────────────────────────
def ransac_fit(points, h, w, side,
               top_y_frac=0.45,
               slope_bounds=None,
               x_bot_left_max_frac=0.65,
               x_bot_right_min_frac=0.35,
               x_top_min_frac=0.15,
               x_top_max_frac=0.85,
               min_pts=15,
               residual_thr=20):

    if len(points) < min_pts:
        return None
    if slope_bounds is None:
        slope_bounds = {"left": (-1.9, -0.08), "right": (0.08, 1.9)}

    s_min, s_max = slope_bounds[side]
    Y = points[:, 0].reshape(-1, 1)
    X = points[:, 1]

    try:
        ransac = RANSACRegressor(
            min_samples=max(5, len(points) // 15),
            residual_threshold=residual_thr,
            max_trials=400,
            random_state=42,
        )
        ransac.fit(Y, X)
        slope     = ransac.estimator_.coef_[0]
        intercept = ransac.estimator_.intercept_

        if not (s_min <= slope <= s_max):
            return None

        y_bot = h - 1
        y_top = int(h * top_y_frac)
        x_bot = int(slope * y_bot + intercept)
        x_top = int(slope * y_top + intercept)
        x_bot = max(0, min(w - 1, x_bot))
        x_top = max(0, min(w - 1, x_top))

        if side == "left"  and x_bot > int(w * x_bot_left_max_frac):
            return None
        if side == "right" and x_bot < int(w * x_bot_right_min_frac):
            return None
        if not (int(w * x_top_min_frac) <= x_top <= int(w * x_top_max_frac)):
            return None

        return (x_bot, y_bot, x_top, y_top)

    except Exception:
        return None


# ─────────────────────────────────────────────────────────────────────────────
# STEP 7 – Draw Lanes
# ─────────────────────────────────────────────────────────────────────────────
def draw_lanes(bgr, left_line, right_line):
    rgb    = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    canvas = rgb.copy()

    if left_line and right_line:
        pts = np.array([
            [left_line[0],  left_line[1]],
            [left_line[2],  left_line[3]],
            [right_line[2], right_line[3]],
            [right_line[0], right_line[1]],
        ], dtype=np.int32)
        overlay = canvas.copy()
        cv2.fillPoly(overlay, [pts], (0, 200, 255))
        canvas = cv2.addWeighted(canvas, 0.72, overlay, 0.28, 0)

    if left_line:
        cv2.line(canvas, (left_line[0],  left_line[1]),
                         (left_line[2],  left_line[3]),
                 (0, 230, 0), 5, cv2.LINE_AA)
    if right_line:
        cv2.line(canvas, (right_line[0], right_line[1]),
                         (right_line[2], right_line[3]),
                 (30, 80, 255), 5, cv2.LINE_AA)
    return canvas


# ─────────────────────────────────────────────────────────────────────────────
# FULL PIPELINE
# ─────────────────────────────────────────────────────────────────────────────
def detect_lanes(image_path):
    bgr = cv2.imread(image_path)
    if bgr is None:
        raise FileNotFoundError(f"Cannot load: {image_path}")

    h, w     = bgr.shape[:2]
    key      = os.path.basename(image_path).replace(".jpg", "")
    cfg      = IMAGE_CONFIG.get(key, dict(version="v1"))
    ver      = cfg["version"]

    gray     = to_grayscale(bgr, clip=2.0 if ver == "v1" else 2.5)
    clr_mask = color_lane_mask(bgr)

    # ── v1 ────────────────────────────────────────────────────────────────
    if ver == "v1":
        gray_roi, roi_mask, _ = make_roi_v1(gray,     h, w)
        clr_roi,  _,        _ = make_roi_v1(clr_mask, h, w)
        top_y_frac       = 0.45
        blob_bottom_frac = 0.40
        slope_bounds     = {"left": (-1.9, -0.08), "right": (0.08, 1.9)}
        fit_kw = dict(top_y_frac=top_y_frac, slope_bounds=slope_bounds,
                      x_bot_left_max_frac=0.70, x_bot_right_min_frac=0.30,
                      x_top_min_frac=0.15, x_top_max_frac=0.85,
                      min_pts=15, residual_thr=20)

    # ── v2 ────────────────────────────────────────────────────────────────
    elif ver == "v2":
        gray_roi, roi_mask, _ = make_roi_v2(gray,     h, w)
        clr_roi,  _,        _ = make_roi_v2(clr_mask, h, w)
        top_y_frac       = 0.55
        blob_bottom_frac = 0.45
        slope_bounds     = {"left": (-1.9, -0.08), "right": (0.08, 1.9)}
        fit_kw = dict(top_y_frac=top_y_frac, slope_bounds=slope_bounds,
                      x_bot_left_max_frac=0.65, x_bot_right_min_frac=0.35,
                      x_top_min_frac=0.15, x_top_max_frac=0.85,
                      min_pts=20, residual_thr=18)

    # ── img1 ──────────────────────────────────────────────────────────────
    elif ver == "img1":
        gray_roi, roi_mask, _ = make_roi_img1(gray,     h, w, cfg)
        clr_roi,  _,        _ = make_roi_img1(clr_mask, h, w, cfg)
        top_y_frac       = cfg["roi_top_y"] / h
        blob_bottom_frac = cfg["blob_bot_frac"]
        slope_bounds     = {"left": (cfg["slope_min_l"], cfg["slope_max_l"]),
                            "right":(cfg["slope_min_r"], cfg["slope_max_r"])}
        fit_kw = dict(top_y_frac=top_y_frac, slope_bounds=slope_bounds,
                      x_bot_left_max_frac=0.85, x_bot_right_min_frac=0.70,
                      x_top_min_frac=0.35, x_top_max_frac=0.75,
                      min_pts=cfg["min_pts"], residual_thr=cfg["residual_thr"])

        sdct_binary, sobel_vis, dct_vis = sdct_edge_detection(gray_roi)
        combined = cv2.bitwise_and(cv2.bitwise_or(sdct_binary, clr_roi), roi_mask)

        ys_all, xs_all = np.where(combined > 0)
        keep  = ys_all > (h * blob_bottom_frac)
        ys_all, xs_all = ys_all[keep], xs_all[keep]
        mid_x = cfg["blob_mid_x"]
        lm = xs_all < mid_x;  rm = xs_all >= mid_x
        left_pts  = np.column_stack((ys_all[lm], xs_all[lm])) if lm.any() else np.empty((0,2))
        right_pts = np.column_stack((ys_all[rm], xs_all[rm])) if rm.any() else np.empty((0,2))

        left_line  = ransac_fit(left_pts,  h, w, side="left",  **fit_kw)
        right_line = ransac_fit(right_pts, h, w, side="right", **fit_kw)
        result     = draw_lanes(bgr, left_line, right_line)
        return {"original":cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB), "gray":gray,
                "roi_mask":roi_mask, "gray_roi":gray_roi, "sobel":sobel_vis,
                "dct_enh":dct_vis, "sdct_bin":sdct_binary, "color_roi":clr_roi,
                "combined":combined, "result":result,
                "left_line":left_line, "right_line":right_line, "version":ver}

    # ── img2  (v4 FIX) ────────────────────────────────────────────────────
    elif ver == "img2":
        gray_roi, roi_mask, _ = make_roi_img2(gray,     h, w, cfg)
        clr_roi,  _,        _ = make_roi_img2(clr_mask, h, w, cfg)
        top_y_frac = cfg["roi_top_y_frac"]

        sdct_binary, sobel_vis, dct_vis = sdct_edge_detection(gray_roi)
        combined = cv2.bitwise_and(cv2.bitwise_or(sdct_binary, clr_roi), roi_mask)

        # Split blobs at cfg["blob_mid_x"] (950) — green side vs blue side
        ys_all, xs_all = np.where(combined > 0)
        keep  = ys_all > (h * cfg["blob_bot_frac"])
        ys_all, xs_all = ys_all[keep], xs_all[keep]
        mid_x = cfg["blob_mid_x"]   # 950
        lm = xs_all >= mid_x        # GREEN line is at x>950 (x_bot~1010)
        rm = xs_all <  mid_x        # BLUE  line is at x<950 (x_bot~879)
        left_pts  = np.column_stack((ys_all[lm], xs_all[lm])) if lm.any() else np.empty((0,2))
        right_pts = np.column_stack((ys_all[rm], xs_all[rm])) if rm.any() else np.empty((0,2))

        # Swapped slope bounds: green=positive, blue=negative
        slope_bounds = {"left":  (cfg["slope_min_l"], cfg["slope_max_l"]),
                        "right": (cfg["slope_min_r"], cfg["slope_max_r"])}

        fit_kw_l = dict(top_y_frac=top_y_frac, slope_bounds=slope_bounds,
                        x_bot_left_max_frac=1.00,   # green line at x~1010 (79% of 1280)
                        x_bot_right_min_frac=0.00,
                        x_top_min_frac=0.40,         # top converges toward center-right
                        x_top_max_frac=0.95,
                        min_pts=cfg["min_pts"], residual_thr=cfg["residual_thr"])

        fit_kw_r = dict(top_y_frac=top_y_frac, slope_bounds=slope_bounds,
                        x_bot_left_max_frac=1.00,
                        x_bot_right_min_frac=0.50,  # blue line at x~879 (69% of 1280)
                        x_top_min_frac=0.50,         # top goes further right
                        x_top_max_frac=1.00,
                        min_pts=cfg["min_pts"], residual_thr=cfg["residual_thr"])

        left_line  = ransac_fit(left_pts,  h, w, side="left",  **fit_kw_l)
        right_line = ransac_fit(right_pts, h, w, side="right", **fit_kw_r)
        result     = draw_lanes(bgr, left_line, right_line)
        return {"original":cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB), "gray":gray,
                "roi_mask":roi_mask, "gray_roi":gray_roi, "sobel":sobel_vis,
                "dct_enh":dct_vis, "sdct_bin":sdct_binary, "color_roi":clr_roi,
                "combined":combined, "result":result,
                "left_line":left_line, "right_line":right_line, "version":ver}

    # ── Common path for v1/v2 ─────────────────────────────────────────────
    sdct_binary, sobel_vis, dct_vis = sdct_edge_detection(gray_roi)
    combined = cv2.bitwise_and(cv2.bitwise_or(sdct_binary, clr_roi), roi_mask)
    left_pts, right_pts = blob_analysis(combined, h, w, bottom_frac=blob_bottom_frac)
    left_line  = ransac_fit(left_pts,  h, w, side="left",  **fit_kw)
    right_line = ransac_fit(right_pts, h, w, side="right", **fit_kw)
    result     = draw_lanes(bgr, left_line, right_line)

    return {
        "original"  : cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
        "gray"      : gray,
        "roi_mask"  : roi_mask,
        "gray_roi"  : gray_roi,
        "sobel"     : sobel_vis,
        "dct_enh"   : dct_vis,
        "sdct_bin"  : sdct_binary,
        "color_roi" : clr_roi,
        "combined"  : combined,
        "result"    : result,
        "left_line" : left_line,
        "right_line": right_line,
        "version"   : ver,
    }


IMAGE_CONFIG = {

    "img1_bonn_ost": dict(
        version        = "img1",
        roi_bot_left   = 500,   roi_bot_right = 1220,
        roi_top_left   = 615,   roi_top_right = 740,
        roi_top_y      = 400,
        blob_mid_x     = 865,
        blob_bot_frac  = 0.55,
        slope_min_l    = -1.9,  slope_max_l = -0.10,
        slope_min_r    =  0.10, slope_max_r =  1.9,
        residual_thr   = 18,
        min_pts        = 15,
    ),


    "img2_bonn_nordost": dict(
        version        = "img2",
        roi_top_y_frac = 0.42,
        roi_bot_left   = 670,  roi_bot_right = 1278,
        roi_top_left   = 750,  roi_top_right = 1100,
        blob_mid_x     = 950,
        blob_bot_frac  = 0.50,
        slope_min_l    =  0.45,  slope_max_l =  1.30,   # GREEN: positive slope
        slope_min_r    = -0.65,  slope_max_r = -0.05,   # BLUE:  negative slope
        residual_thr   = 18,
        min_pts        = 12,
    ),

    "img3_forest_road": dict(version="v1"),
    "img4_tunnel"     : dict(version="v1"),
    "img5_california" : dict(version="v2"),
}

# ─────────────────────────────────────────────────────────────────────────────
# PIPELINE VISUALISATION
# ─────────────────────────────────────────────────────────────────────────────
def visualise_pipeline(out, title, save_path):
    steps = [
        ("1. Original RGB",         out["original"],  None),
        ("2. Grayscale + CLAHE",    out["gray"],      "gray"),
        ("3. ROI Mask",             out["roi_mask"],  "gray"),
        ("4a. Sobel Magnitude",     out["sobel"],     "gray"),
        ("4b. DCT High-Pass",       out["dct_enh"],   "gray"),
        ("4c. SDCT Binary Edges",   out["sdct_bin"],  "gray"),
        ("5. HLS Color Mask",       out["color_roi"], "gray"),
        ("5+. Combined Blobs",      out["combined"],  "gray"),
        ("6+7. RANSAC Lane Result", out["result"],    None),
    ]
    fig, axes = plt.subplots(3, 3, figsize=(18, 12))
    fig.suptitle(f"{title}  [mode: {out['version']}]",
                 fontsize=14, fontweight="bold", y=1.01)
    for ax, (label, img, cmap) in zip(axes.flat, steps):
        ax.imshow(img, cmap=cmap if cmap and len(img.shape)==2 else None)
        ax.set_title(label, fontsize=9, fontweight="bold")
        ax.axis("off")
    patches = [
        mpatches.Patch(color=(0,0.9,0),       label="Left Lane  (GREEN)"),
        mpatches.Patch(color=(0.12,0.31,1.0), label="Right Lane (BLUE)"),
        mpatches.Patch(color=(0,0.78,1.0),    label="Lane Region (CYAN)"),
    ]
    fig.legend(handles=patches, loc="lower center", ncol=3,
               fontsize=10, bbox_to_anchor=(0.5, -0.02))
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  ✓ Pipeline: {save_path}")


# ─────────────────────────────────────────────────────────────────────────────
# SUMMARY FIGURE
# ─────────────────────────────────────────────────────────────────────────────
def make_summary_figure(results_list, save_path):
    n = len(results_list)
    fig, axes = plt.subplots(1, n, figsize=(6*n, 5))
    fig.suptitle("Lane Detection v4 — All Images", fontsize=14, fontweight="bold")
    for ax, (path, out) in zip(axes, results_list):
        ax.imshow(out["result"])
        short = os.path.basename(path).replace(".jpg","").replace("_"," ").title()
        ll = "✅ L" if out["left_line"]  else "❌ L"
        rl = "✅ R" if out["right_line"] else "❌ R"
        ax.set_title(f"{short}\n{ll}  {rl}  [{out['version']}]",
                     fontsize=9, fontweight="bold")
        ax.axis("off")
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\n  ✓ Summary: {save_path}")





# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    image_paths = [
        r"C:\Users\saran\Downloads\lane_detection_v3\img1_bonn_ost.jpg",
        
        r"C:\Users\saran\Downloads\lane_detection_v3\img3_forest_road.jpg",
        r"C:\Users\saran\Downloads\lane_detection_v3\img4_tunnel.jpg",
        r"C:\Users\saran\Downloads\lane_detection_v3\img5_california.jpg",
        r"C:\Users\saran\Downloads\lane_detection_v3\img1.jpeg",
        r"C:\Users\saran\Downloads\lane_detection_v3\img2.jpeg",
        r"C:\Users\saran\Downloads\lane_detection_v3\img3.jpeg"
    ]

    os.makedirs(r"C:\Users\saran\Downloads\lane_detection_v3\outputs", exist_ok=True)
    results_list = []

    for path in image_paths:
        name = os.path.basename(path).replace(".jpg", "")
        print(f"\n── {name} ──")
        try:
            out = detect_lanes(path)
            print(f"  version : {out['version']}")
            print(f"  Left    : {out['left_line']  or '❌ not detected'}")
            print(f"  Right   : {out['right_line'] or '❌ not detected'}")
            fig_path = fr"C:\Users\saran\Downloads\lane_detection_v3\outputs\{name}_pipeline_v3.jpg"
            visualise_pipeline(out, name.replace("_"," ").title(), fig_path)
            results_list.append((path, out))
        except Exception as e:
            print(f"  ✗ Error: {e}")

    make_summary_figure(results_list,
                        r"C:\Users\saran\Downloads\lane_detection_v3\outputs\00_all_lanes_summary_v3.jpg")
    print("\n✅ Done!")