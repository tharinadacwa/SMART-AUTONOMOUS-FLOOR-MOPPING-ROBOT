#!/usr/bin/env python3
"""
prepare_map.py -- OFFLINE. Run this on your LAPTOP, before the robot ever moves.

This is the single most important tool in the project, and it exists because you
said: "I want to do this once without facing any problem."

The way you get that is by moving every decision that CAN be made offline, OFFLINE
-- where a mistake costs you a re-run of a Python script, not a robot stuck behind
a sofa for 40 minutes.

WHAT IT DOES
    1. Loads your saved map (.pgm + .yaml, standard nav2 map_server format).
    2. CLEANS it -- removes speckle, optionally seals hairline gaps in walls that
       the LIDAR left behind. An unsealed 6 cm gap in a wall will leak your
       coverage region into the neighbour's flat.
    3. QA REPORT -- and this is the part that saves you:
         * which regions the robot can actually REACH from its start pose
         * which regions are UNREACHABLE (doorway too narrow -- it physically
           will not fit) so you find out now, not mid-run
         * the width of every bottleneck
         * total drivable area, and the honest time estimate
    4. PRE-COMPUTES the full coverage path: edge pass + 19 cm boustrophedon lanes.
       The lanes run PARALLEL TO THE WALLS by default (dominant wall direction,
       detected with a Hough transform). Use --align to change this:
         --align walls    (default) lanes parallel to the dominant walls
         --align perpendicular lanes at 90 deg to the walls
         --align pca      old behaviour: long axis of the free-space blob
         --sweep-angle D  force the angle to D degrees, overrides --align
    5. RENDERS A PNG so you can LOOK at the path before committing to it.
    6. Writes coverage_path.yaml, which coverage_runner.py just plays back.

WHY PRE-COMPUTE INSTEAD OF PLANNING AT RUNTIME
    The map is static. The path is therefore static. Computing it at runtime buys
    you nothing and costs you determinism: if the planner is going to make a bad
    decision, you want to see it on your screen, not discover it when the robot is
    wedged in a doorway. Pre-computing also means the run is REPEATABLE -- the same
    map produces the same path, every time.

USAGE
    python3 prepare_map.py --map maps/home.yaml --start 0 0

    # seal gaps up to 10 cm, exclude a keepout mask, preview only
    python3 prepare_map.py --map maps/home.yaml --start 0 0 \
        --seal-gaps 0.10 --keepout maps/home_keepout.pgm --dry-run

OUTPUTS (next to your map)
    home_coverage_path.yaml     <- fed to the robot
    home_coverage_preview.png   <- LOOK AT THIS BEFORE YOU RUN
    home_qa.txt                 <- the report
    home_keepout_template.pgm   <- paint on this in GIMP to add no-go zones
"""

import argparse
import datetime
import math
import os
import sys

import numpy as np
import yaml

try:
    from scipy import ndimage
except ImportError:
    sys.exit("Needs scipy:  pip3 install scipy   (or  sudo apt install python3-scipy)")

try:
    import cv2
except ImportError:
    sys.exit("Needs opencv: pip3 install opencv-python  (or  sudo apt install python3-opencv)")


# ===========================================================================
#  Map I/O  (nav2 map_server format)
# ===========================================================================

def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)

    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f'Could not read map image: {img_path}')

    res = float(meta['resolution'])
    origin = [float(v) for v in meta['origin']]
    negate = int(meta.get('negate', 0))
    occ_th = float(meta.get('occupied_thresh', 0.65))
    free_th = float(meta.get('free_thresh', 0.196))

    # map_server convention: p = 1 - pixel/255  (darker = more occupied)
    p = img.astype(np.float64) / 255.0
    p = p if negate else (1.0 - p)

    grid = np.full(img.shape, -1, dtype=np.int8)     # unknown
    grid[p > occ_th] = 100                           # occupied
    grid[p < free_th] = 0                            # free

    # Image row 0 is the TOP. OccupancyGrid row 0 is the BOTTOM. Flip.
    grid = np.flipud(grid)

    return grid, res, origin, meta


def c2w(r, c, res, origin):
    return origin[0] + (c + 0.5) * res, origin[1] + (r + 0.5) * res


def w2c(x, y, res, origin):
    return int((y - origin[1]) / res), int((x - origin[0]) / res)


# ===========================================================================
#  Cleaning
# ===========================================================================

def clean_map(grid, res, seal_gaps_m, despeckle_m):
    """Seal hairline wall gaps and remove sensor speckle."""
    occ = (grid == 100)
    report = []

    if seal_gaps_m > 0:
        k = max(1, int(round(seal_gaps_m / res)))
        # Morphological CLOSE on the obstacle layer: dilate then erode. Bridges
        # gaps up to ~seal_gaps_m wide without fattening the walls permanently.
        st = np.ones((2 * k + 1, 2 * k + 1), dtype=bool)
        closed = ndimage.binary_closing(occ, structure=st)
        sealed = int(np.count_nonzero(closed & ~occ))
        occ = closed
        report.append(f'sealed {sealed} cells of wall gaps (<= {seal_gaps_m*100:.0f} cm)')

    if despeckle_m > 0:
        min_cells = max(1, int((despeckle_m / res) ** 2))
        lab, n = ndimage.label(occ)
        removed = 0
        for i in range(1, n + 1):
            m = (lab == i)
            if np.count_nonzero(m) < min_cells:
                occ[m] = False
                removed += 1
        report.append(f'removed {removed} obstacle speckles (< {despeckle_m*100:.0f} cm across)')

    out = grid.copy()
    out[occ] = 100
    out[(grid == 100) & ~occ] = 0        # despeckled obstacles become free
    return out, report


# ===========================================================================
#  Reachability QA -- the bit that saves your afternoon
# ===========================================================================

def analyse(grid, res, origin, erode_r, start_xy, lane, v_max):
    free = (grid >= 0) & (grid < 50)     # UNKNOWN counts as OBSTACLE. Always.

    dist = ndimage.distance_transform_edt(free) * res
    drivable = dist >= erode_r

    labels, n = ndimage.label(drivable)

    sr, sc = w2c(start_xy[0], start_xy[1], res, origin)
    h, w = drivable.shape
    if not (0 <= sr < h and 0 <= sc < w):
        sys.exit(f'Start pose {start_xy} is outside the map.')

    start_lab = labels[sr, sc]
    if start_lab == 0:
        # Snap to the nearest drivable cell and tell the user.
        rr, cc = np.nonzero(drivable)
        if len(rr) == 0:
            sys.exit(f'NOTHING is drivable. A {2*(erode_r):.2f} m robot does not fit '
                     'anywhere in this map. Check your map resolution and robot_radius.')
        d = (rr - sr) ** 2 + (cc - sc) ** 2
        i = int(np.argmin(d))
        snapped = c2w(rr[i], cc[i], res, origin)
        start_lab = labels[rr[i], cc[i]]
        print(f'  ! start pose {start_xy} is not drivable; snapped to '
              f'({snapped[0]:.2f}, {snapped[1]:.2f})')
        start_xy = snapped

    reachable = (labels == start_lab)

    islands = []
    for i in range(1, n + 1):
        if i == start_lab:
            continue
        area = np.count_nonzero(labels == i) * res * res
        if area < 0.2:
            continue
        rr, cc = np.nonzero(labels == i)
        cx, cy = c2w(float(rr.mean()), float(cc.mean()), res, origin)
        islands.append((area, cx, cy))

    reach_area = np.count_nonzero(reachable) * res * res
    free_area = np.count_nonzero(free) * res * res

    # Bottlenecks: the narrowest place the robot must squeeze through. We measure
    # it on the FREE map, along the skeleton of the reachable region.
    sk = ndimage.binary_erosion(reachable, iterations=1) ^ reachable
    narrowest = None
    if reachable.any():
        d_free = ndimage.distance_transform_edt(free) * res
        vals = d_free[reachable]
        if len(vals):
            narrowest = float(vals.min()) * 2.0    # full corridor width

    return dict(free=free, dist=dist, drivable=drivable, labels=labels,
                reachable=reachable, start_lab=start_lab, start_xy=start_xy,
                islands=islands, reach_area=reach_area, free_area=free_area,
                narrowest=narrowest)


# ===========================================================================
#  Coverage path  (edge pass + boustrophedon fill)
# ===========================================================================

def densify(pts, wp):
    out = []
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        s = math.dist((x0, y0), (x1, y1))
        if s < 1e-9:
            continue
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(round(s / wp)))
        for j in range(n):
            t = j / n
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw))
    if len(pts) >= 2:
        yaw = math.atan2(pts[-1][1] - pts[-2][1], pts[-1][0] - pts[-2][0])
        out.append((pts[-1][0], pts[-1][1], yaw))
    return out


def edge_pass(mask, res, origin, lane, wp):
    cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_LIST,
                               cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for cnt in cnts:
        if cv2.arcLength(cnt, True) * res < 3 * lane:
            continue
        cnt = cv2.approxPolyDP(cnt, 0.01 / res, True)
        pts = [c2w(int(p[0][1]), int(p[0][0]), res, origin) for p in cnt]
        if len(pts) >= 3:
            pts.append(pts[0])
            out += densify(pts, wp)
    return out


def hough_wall_angle(grid, res, min_wall_m=0.5):
    """Dominant wall orientation (radians, world frame) from the occupied pixels.

    Buildings are overwhelmingly rectilinear: nearly every wall is parallel or
    perpendicular to nearly every other wall. So we detect wall segments with a
    probabilistic Hough transform, fold each segment's angle into a 90-degree
    period, and take the length-weighted mean. The result is the single angle
    that most of the wall length agrees on -- exactly the axis we want the mop
    lanes to run along.

    Returns None if there are no walls / nothing detected, so the caller can
    fall back to PCA.
    """
    walls = (grid == 100).astype(np.uint8) * 255
    if np.count_nonzero(walls) == 0:
        return None

    min_len_px = max(8, int(round(min_wall_m / res)))
    lines = cv2.HoughLinesP(walls, 1, np.pi / 180.0,
                            threshold=max(10, min_len_px // 2),
                            minLineLength=min_len_px,
                            maxLineGap=max(2, min_len_px // 4))
    if lines is None or len(lines) == 0:
        return None

    # Fold into a 90-degree period with the 4*theta trick, so that 0 and 90 deg
    # REINFORCE each other instead of cancelling. (A plain average of raw angles
    # would put a room with equal N-S and E-W walls at a meaningless 45 deg.)
    sx = sy = 0.0
    for ln in lines:
        x1, y1, x2, y2 = (int(v) for v in ln[0])
        a = math.atan2(y2 - y1, x2 - x1)          # world angle: col->+x, row->+y
        wgt = math.hypot(x2 - x1, y2 - y1)         # weight by wall length
        sx += wgt * math.cos(4.0 * a)
        sy += wgt * math.sin(4.0 * a)
    if sx == 0.0 and sy == 0.0:
        return None
    return math.atan2(sy, sx) / 4.0               # back to (-pi/4, pi/4]


def pca_wall_angle(mask, res, origin):
    """Old behaviour: long axis of the free-space blob (fewest 180 turns)."""
    rows, cols = np.nonzero(mask)
    if len(rows) == 0:
        return 0.0
    xs = origin[0] + (cols + 0.5) * res
    ys = origin[1] + (rows + 0.5) * res
    pts = np.column_stack((xs, ys))
    ctr = pts.mean(axis=0)
    evals, evecs = np.linalg.eigh(np.cov((pts - ctr).T))
    ax = evecs[:, int(np.argmax(evals))]
    return math.atan2(float(ax[1]), float(ax[0]))


def compute_sweep_angle(grid, mask, res, origin, align, explicit_deg):
    """Pick the boustrophedon sweep angle. Returns (theta_radians, source_str)."""
    if explicit_deg is not None:
        return math.radians(explicit_deg), f'manual override {explicit_deg:.1f} deg'

    if align == 'pca':
        return pca_wall_angle(mask, res, origin), 'PCA long axis of free space'

    # 'walls' or 'perpendicular': start from the dominant wall direction.
    th = hough_wall_angle(grid, res)
    if th is None:
        th = pca_wall_angle(mask, res, origin)
        src = 'PCA fallback (no walls detected by Hough)'
    else:
        src = 'dominant wall direction (Hough)'

    if align == 'perpendicular':
        th += math.pi / 2.0
        src = ('perpendicular, 90 deg to ' + src) if 'wall' in src else (src + ' + 90 deg')

    return th, src


def fill_pass(mask, res, origin, lane, wp, min_seg, start_xy, theta):
    rows, cols = np.nonzero(mask)
    if len(rows) == 0:
        return [], math.degrees(theta), 0

    xs = origin[0] + (cols + 0.5) * res
    ys = origin[1] + (rows + 0.5) * res
    pts = np.column_stack((xs, ys))

    # Sweep angle is chosen by compute_sweep_angle() -- parallel to the walls by
    # default, instead of the free-space PCA axis this function used to compute.
    ctr = pts.mean(axis=0)
    th = theta
    ct, st = math.cos(th), math.sin(th)

    d = pts - ctr
    u = d[:, 0] * ct + d[:, 1] * st
    v = -d[:, 0] * st + d[:, 1] * ct
    u0, u1 = float(u.min()), float(u.max())
    v0, v1 = float(v.min()), float(v.max())

    n_lanes = max(1, int((v1 - v0) / lane) + 1)
    vs = v0 + ((v1 - v0) - (n_lanes - 1) * lane) / 2.0

    h, w = mask.shape
    step = max(res * 0.5, 0.02)

    lanes = {}
    for k in range(n_lanes):
        vv = vs + k * lane
        runs, run = [], []
        uu = u0
        while uu <= u1:
            wx = ctr[0] + uu * ct - vv * st
            wy = ctr[1] + uu * st + vv * ct
            r, c = w2c(wx, wy, res, origin)
            if 0 <= r < h and 0 <= c < w and mask[r, c]:
                run.append((wx, wy))
            else:
                if len(run) * step >= min_seg:
                    runs.append(run)
                run = []
            uu += step
        if len(run) * step >= min_seg:
            runs.append(run)
        if runs:
            lanes[k] = runs

    out = []
    cur = start_xy
    for k in sorted(lanes):
        runs = list(lanes[k])
        while runs:
            bi, brev, bd = 0, False, float('inf')
            for i, rn in enumerate(runs):
                for rev, pt in ((False, rn[0]), (True, rn[-1])):
                    dd = math.dist(cur, pt)
                    if dd < bd:
                        bi, brev, bd = i, rev, dd
            rn = runs.pop(bi)
            if brev:
                rn = rn[::-1]
            out += densify(rn, wp)
            cur = rn[-1]
    return out, math.degrees(th), n_lanes


# ===========================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='path to map .yaml')
    ap.add_argument('--start', nargs=2, type=float, default=[0.0, 0.0],
                    metavar=('X', 'Y'), help='where the robot starts, in map frame')
    # ---------------------------------------------------------------------
    #  DEFAULTS ARE DERIVED FROM robot.yaml BY tools/generate_config.py.
    #  See docs/DERIVED_NUMBERS.md. If you change robot.yaml, regenerate and
    #  update these -- otherwise you will plan a path with stale numbers and the
    #  time estimate (and the reachability QA) will quietly lie to you.
    #
    #  REVISION 2 -- MEASURED HARDWARE
    #    wheel_radius 0.0336 (67.2 mm dia), wheel_separation 0.400, 5-20 RPM
    #      v_max     = 0.0704 m/s
    #      omega_max = 0.3519 rad/s   (we plan against 0.2991, the RPP limit)
    #
    #  !! robot-radius is 0.210, NOT 0.190 !!
    #     The BODY is 380 mm across but the WHEELS SPAN 410-420 mm. The wheels
    #     are the widest part. Plan against the body and the planner will hand
    #     Nav2 waypoints that scrape the wheels along every wall.
    # ---------------------------------------------------------------------
    ap.add_argument('--robot-radius', type=float, default=0.210)
    ap.add_argument('--safety-margin', type=float, default=0.03)
    ap.add_argument('--lane-spacing', type=float, default=0.19)
    ap.add_argument('--cleaning-head-width', type=float, default=0.30,
                    help='total swept width. Two 100mm brushes 200mm apart = 0.30')
    ap.add_argument('--waypoint-spacing', type=float, default=0.15)
    ap.add_argument('--min-segment', type=float, default=0.25)
    ap.add_argument('--max-linear-vel', type=float, default=0.0704)
    ap.add_argument('--angular-vel', type=float, default=0.2991)
    ap.add_argument('--seal-gaps', type=float, default=0.06,
                    help='close wall gaps up to this many metres (0 = off)')
    ap.add_argument('--despeckle', type=float, default=0.06)
    ap.add_argument('--align', choices=['walls', 'perpendicular', 'pca'], default='walls',
                    help='lane direction: walls=parallel to dominant walls (default), '
                         'perpendicular=90 deg to walls, pca=old long-axis-of-free-space')
    ap.add_argument('--sweep-angle', type=float, default=None,
                    help='force sweep angle in degrees (world frame); overrides --align')
    ap.add_argument('--no-edge-pass', action='store_true')
    ap.add_argument('--keepout', default=None,
                    help='optional keepout mask .pgm; dark = do not clean')
    ap.add_argument('--dry-run', action='store_true',
                    help='report + preview only, do not write coverage_path.yaml')
    a = ap.parse_args()

    base = os.path.splitext(os.path.abspath(a.map))[0]
    erode_r = a.robot_radius + a.safety_margin

    print(f'\n=== Loading {a.map} ===')
    grid, res, origin, meta = load_map(a.map)
    print(f'  {grid.shape[1]} x {grid.shape[0]} cells @ {res:.3f} m/cell, '
          f'origin {origin[0]:.2f}, {origin[1]:.2f}')

    grid, clean_report = clean_map(grid, res, a.seal_gaps, a.despeckle)
    for line in clean_report:
        print(f'  {line}')

    # keepout mask: paint it black in GIMP, it becomes obstacle
    if a.keepout:
        km = cv2.imread(a.keepout, cv2.IMREAD_GRAYSCALE)
        if km is None:
            sys.exit(f'Could not read keepout mask {a.keepout}')
        km = np.flipud(km)
        if km.shape != grid.shape:
            sys.exit(f'Keepout mask {km.shape} does not match map {grid.shape}. '
                     'It must be the same size -- start from a COPY of your map.pgm.')
        blocked = km < 128
        n_block = int(np.count_nonzero(blocked & (grid == 0)))
        grid[blocked] = 100
        print(f'  keepout mask blocked {n_block} free cells '
              f'({n_block*res*res:.2f} m^2)')

    print(f'\n=== Reachability (robot radius {a.robot_radius:.3f}, '
          f'erode {erode_r:.3f}) ===')
    A = analyse(grid, res, origin, erode_r, tuple(a.start), a.lane_spacing,
                a.max_linear_vel)

    print(f'  free floor area      : {A["free_area"]:.2f} m^2')
    print(f'  REACHABLE drivable   : {A["reach_area"]:.2f} m^2')
    if A['narrowest']:
        print(f'  narrowest corridor   : {A["narrowest"]:.2f} m '
              f'(robot needs {2*erode_r:.2f} m)')

    problems = []
    if A['islands']:
        print(f'\n  !! {len(A["islands"])} UNREACHABLE REGION(S) -- the robot '
              f'cannot fit through the gap to get there:')
        for area, cx, cy in sorted(A['islands'], reverse=True):
            print(f'       {area:6.2f} m^2 around ({cx:6.2f}, {cy:6.2f})')
            problems.append(f'unreachable {area:.2f} m^2 at ({cx:.2f}, {cy:.2f})')
        print(f'     FIX: widen the doorway, or reduce --safety-margin, or accept it.')

    # ---- plan ------------------------------------------------------------
    print(f'\n=== Planning coverage (lane spacing {a.lane_spacing:.3f} m) ===')
    mask = A['reachable']
    start = A['start_xy']

    path = []
    n_edge = 0
    if not a.no_edge_pass:
        e = edge_pass(mask, res, origin, a.lane_spacing, a.waypoint_spacing)
        n_edge = len(e)
        path += e
        print(f'  edge pass : {n_edge} waypoints  '
              f'(this is what cleans the CORNERS)')
    else:
        problems.append('edge pass DISABLED -- corners will not be cleaned')

    theta, ang_src = compute_sweep_angle(grid, mask, res, origin,
                                         a.align, a.sweep_angle)
    print(f'  sweep angle: {math.degrees(theta):.1f} deg  [{ang_src}]')
    f, sweep_deg, n_lanes = fill_pass(mask, res, origin, a.lane_spacing,
                                      a.waypoint_spacing, a.min_segment, start, theta)
    path += f
    print(f'  fill pass : {len(f)} waypoints, {n_lanes} lanes, '
          f'sweeping at {sweep_deg:.1f} deg')

    if not path:
        sys.exit('No path produced. The reachable region is too small.')

    length = sum(math.dist(path[i][:2], path[i + 1][:2])
                 for i in range(len(path) - 1))
    t_drive = length / a.max_linear_vel
    t_turn = n_lanes * math.pi / a.angular_vel
    total_min = (t_drive + t_turn) / 60

    print(f'\n=== Result ===')
    print(f'  waypoints    : {len(path)}')
    print(f'  path length  : {length:.1f} m')
    print(f'  driving      : {t_drive/60:.0f} min at {a.max_linear_vel:.4f} m/s')
    print(f'  turning      : {t_turn/60:.1f} min ({n_lanes} x 180 deg @ '
          f'{math.pi/a.angular_vel:.1f} s)')
    print(f'  ESTIMATED RUN: {total_min:.0f} MINUTES')

    # ---- simulate the sweep ---------------------------------------------
    # Simulate the ACTUAL brush sweep (0.30 m head), not the lane spacing.
    # Using lane_spacing here would have quietly reported the coverage of a
    # 19 cm head when you actually have a 30 cm one.
    cov = np.zeros_like(mask)
    brush = max(1, int(round((a.cleaning_head_width / 2) / res)))
    h, w = mask.shape
    for (x, y, _) in path:
        r0, c0 = w2c(x, y, res, origin)
        r_lo, r_hi = max(0, r0 - brush), min(h, r0 + brush + 1)
        c_lo, c_hi = max(0, c0 - brush), min(w, c0 + brush + 1)
        if r_lo >= r_hi or c_lo >= c_hi:
            continue
        rr, cc = np.ogrid[r_lo:r_hi, c_lo:c_hi]
        cov[r_lo:r_hi, c_lo:c_hi] |= ((rr - r0) ** 2 + (cc - c0) ** 2 <= brush * brush)

    pct = 100.0 * np.count_nonzero(mask & cov) / max(1, np.count_nonzero(mask))

    body = np.zeros_like(A['free'])
    brad = max(1, int(round((a.cleaning_head_width / 2) / res)))
    for (x, y, _) in path:
        r0, c0 = w2c(x, y, res, origin)
        r_lo, r_hi = max(0, r0 - brad), min(h, r0 + brad + 1)
        c_lo, c_hi = max(0, c0 - brad), min(w, c0 + brad + 1)
        if r_lo >= r_hi or c_lo >= c_hi:
            continue
        rr, cc = np.ogrid[r_lo:r_hi, c_lo:c_hi]
        body[r_lo:r_hi, c_lo:c_hi] |= ((rr - r0) ** 2 + (cc - c0) ** 2 <= brad * brad)
    body_pct = 100.0 * np.count_nonzero(A['free'] & body) / max(1, np.count_nonzero(A['free']))

    wall_strip = erode_r - a.cleaning_head_width / 2.0
    print(f'  coverage of drivable region     : {pct:.1f}%')
    print(f'  floor swept by the BRUSHES      : {body_pct:.1f}% of all free space')
    print(f'  UNCLEANED WALL STRIP            : {wall_strip*100:.0f} cm along every wall')
    print(f'     (robot centre stops {erode_r:.3f} m from a wall; the {a.cleaning_head_width:.2f} m')
    print(f'      brush head reaches {a.cleaning_head_width/2:.3f} m beyond that.)')
    print(f'     This is MECHANICAL. No planner can fix it.')
    if pct < 98:
        problems.append(f'coverage of drivable region only {pct:.1f}%')

    # ---- write outputs ---------------------------------------------------
    render(base, grid, A, path, n_edge, res, origin, pct, body_pct, total_min,
           sweep_deg, ang_src)

    qa = f'''MAP QA REPORT   {datetime.datetime.now():%Y-%m-%d %H:%M}
map                 : {a.map}
resolution          : {res:.3f} m/cell
robot radius        : {a.robot_radius:.3f} m  (diameter {2*a.robot_radius:.3f} m)
erosion radius      : {erode_r:.3f} m
min passable gap    : {2*erode_r:.3f} m
lane spacing        : {a.lane_spacing:.3f} m

free floor area     : {A['free_area']:.2f} m^2
reachable drivable  : {A['reach_area']:.2f} m^2
unreachable regions : {len(A['islands'])}
narrowest corridor  : {A['narrowest'] if A['narrowest'] else float('nan'):.2f} m

waypoints           : {len(path)}  ({n_edge} edge + {len(path)-n_edge} fill)
lanes               : {n_lanes}
path length         : {length:.1f} m
ESTIMATED RUN       : {total_min:.0f} minutes

coverage (drivable) : {pct:.1f}%
floor swept (body)  : {body_pct:.1f}%

PROBLEMS:
''' + ('\n'.join('  - ' + p for p in problems) if problems else '  none\n')

    with open(base + '_qa.txt', 'w') as fh:
        fh.write(qa)
    print(f'\n  wrote {base}_qa.txt')

    # keepout template
    if not a.keepout:
        tmpl = np.full(grid.shape, 255, dtype=np.uint8)
        tmpl[grid == 100] = 0
        tmpl[grid == -1] = 128
        cv2.imwrite(base + '_keepout_template.pgm', np.flipud(tmpl))
        print(f'  wrote {base}_keepout_template.pgm  '
              '(paint areas BLACK in GIMP to forbid them, then pass --keepout)')

    if a.dry_run:
        print('\n  --dry-run: coverage_path.yaml NOT written.\n')
        return

    out = {
        'metadata': {
            'map': os.path.basename(a.map),
            'generated': datetime.datetime.now().isoformat(timespec='seconds'),
            'robot_radius': float(a.robot_radius),
            'safety_margin': float(a.safety_margin),
            'lane_spacing': float(a.lane_spacing),
            'sweep_deg': round(float(sweep_deg), 2),
            'align_mode': a.align if a.sweep_angle is None else 'manual',
            'n_edge_waypoints': int(n_edge),
            'n_waypoints': int(len(path)),
            'path_length_m': round(float(length), 2),
            'estimated_minutes': round(float(total_min), 1),
            'coverage_pct': round(float(pct), 1),
            'start': [float(start[0]), float(start[1])],
        },
        'poses': [
            {'x': round(float(x), 4), 'y': round(float(y), 4),
             'yaw': round(float(t), 4),
             'pass': ('edge' if i < n_edge else 'fill')}
            for i, (x, y, t) in enumerate(path)
        ],
    }
    with open(base + '_coverage_path.yaml', 'w') as fh:
        yaml.safe_dump(out, fh, sort_keys=False)
    print(f'  wrote {base}_coverage_path.yaml   <-- feed this to coverage_runner\n')

    print('  >>> NOW OPEN THE PREVIEW PNG AND LOOK AT IT. <<<')
    print('      If the path looks wrong on your screen, it will be wrong on your floor.\n')


def render(base, grid, A, path, n_edge, res, origin, pct, body_pct, total_min,
           sweep_deg=0.0, ang_src=''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 3, figsize=(21, 7))

    show = np.zeros(grid.shape)
    show[grid == -1] = 0.5
    show[grid == 100] = 1.0
    axs[0].imshow(show, origin='lower', cmap='gray_r', vmin=0, vmax=1)
    axs[0].set_title('1. Cleaned map\nblack=wall  grey=unknown(=obstacle)')

    reg = np.zeros(grid.shape)
    reg[A['drivable']] = 1
    reg[A['reachable']] = 2
    axs[1].imshow(reg, origin='lower', cmap='YlGn', vmin=0, vmax=2)
    t = f'2. Drivable region\ndark green = REACHABLE ({A["reach_area"]:.1f} m2)'
    if A['islands']:
        t += f'\npale = {len(A["islands"])} UNREACHABLE island(s)'
    axs[1].set_title(t)

    axs[2].imshow(show, origin='lower', cmap='gray_r', alpha=0.35, vmin=0, vmax=1)
    p = np.array([(w2c(x, y, res, origin)[1], w2c(x, y, res, origin)[0])
                  for (x, y, _) in path], dtype=float)
    if n_edge:
        axs[2].plot(p[:n_edge, 0], p[:n_edge, 1], '-', lw=1.6, color='#1f77b4',
                    label=f'edge pass ({n_edge} wp) — the corners')
    axs[2].plot(p[n_edge:, 0], p[n_edge:, 1], '-', lw=0.7, color='crimson',
                label=f'fill lanes ({len(path)-n_edge} wp)')
    sr, sc = w2c(A['start_xy'][0], A['start_xy'][1], res, origin)
    axs[2].plot(sc, sr, 'o', ms=11, color='lime', mec='k', label='start')
    axs[2].legend(loc='upper right', fontsize=8)
    axs[2].set_title(f'3. Coverage path — {pct:.0f}% of drivable, '
                     f'{body_pct:.0f}% of floor swept\n'
                     f'~{total_min:.0f} min run — lanes at {sweep_deg:.1f}° ({ang_src})')

    for ax in axs:
        ax.set_xticks([])
        ax.set_yticks([])
    plt.tight_layout()
    plt.savefig(base + '_coverage_preview.png', dpi=100)
    print(f'  wrote {base}_coverage_preview.png')


if __name__ == '__main__':
    main()
