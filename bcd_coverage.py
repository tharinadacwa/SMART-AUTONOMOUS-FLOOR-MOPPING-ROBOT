#!/usr/bin/env python3
"""
bcd_coverage.py -- Boustrophedon Cellular Decomposition path generator.

DROP-IN REPLACEMENT for prepare_map.py's path generation.
Writes the SAME yaml schema, so coverage_server.py needs ZERO changes.

WHAT IS DIFFERENT FROM prepare_map.py
  * NO edge/perimeter pass. The robot starts sweeping lanes immediately.
    (metadata n_edge_waypoints is written as 0.)
  * Proper Boustrophedon Cellular Decomposition: the free space is swept with
    a vertical line; where the number of connected slices CHANGES (an obstacle
    starts or ends) that is a critical point and a new cell begins. Each cell
    is then covered with a pure serpentine. This is the classic Choset BCD.
  * Lanes run PARALLEL to the dominant wall direction (Hough), or
    PERPENDICULAR with --align perpendicular.

USAGE (run on the LAPTOP, next to your map)
    python3 bcd_coverage.py --map maps/room_v2.yaml --start 0 0
    python3 bcd_coverage.py --map maps/room_v2.yaml --start 0 0 --align perpendicular
    python3 bcd_coverage.py --map maps/room_v2.yaml --start 0 0 --sweep-angle 0

OUTPUTS (next to the map)
    room_v2_coverage_path.yaml      <- copy this to the Pi
    room_v2_coverage_preview.png    <- LOOK AT THIS FIRST
"""

import argparse, math, os, sys, datetime
import numpy as np
import yaml

try:
    import cv2
except ImportError:
    sys.exit("Needs opencv:  pip3 install opencv-python")
try:
    from scipy import ndimage
except ImportError:
    sys.exit("Needs scipy:  pip3 install scipy")


# ---------------------------------------------------------------- map I/O
def load_map(yaml_path):
    with open(yaml_path) as f:
        m = yaml.safe_load(f)
    d = os.path.dirname(os.path.abspath(yaml_path))
    pgm = m['image'] if os.path.isabs(m['image']) else os.path.join(d, m['image'])
    img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"cannot read {pgm}")
    res = float(m['resolution'])
    origin = [float(v) for v in m['origin']]
    negate = int(m.get('negate', 0))
    occ_th = float(m.get('occupied_thresh', 0.65))
    free_th = float(m.get('free_thresh', 0.196))
    p = img.astype(np.float32) / 255.0
    if not negate:
        p = 1.0 - p                       # now 1.0 = occupied
    free = (p < free_th)                  # definitely free
    occ = (p > occ_th)                    # definitely occupied
    unknown = ~free & ~occ
    return free, occ, unknown, res, origin, img


def world_to_px(x, y, origin, res, h):
    c = int((x - origin[0]) / res)
    r = h - 1 - int((y - origin[1]) / res)
    return r, c


def px_to_world(r, c, origin, res, h):
    x = c * res + origin[0] + res / 2.0
    y = (h - 1 - r) * res + origin[1] + res / 2.0
    return x, y


# ---------------------------------------------------- dominant wall angle
def dominant_wall_angle(occ, res, min_wall_m=0.5):
    """Hough on the obstacle mask -> the angle the walls run at, in radians."""
    edges = (occ.astype(np.uint8)) * 255
    min_len = max(8, int(round(min_wall_m / res)))
    lines = cv2.HoughLinesP(edges, 1, np.pi / 360, threshold=min_len,
                            minLineLength=min_len, maxLineGap=int(min_len / 2))
    if lines is None or len(lines) == 0:
        print("  ! no walls found by Hough -> using 0 deg")
        return 0.0
    # bin angles mod 90 deg (a rectangular room has walls at t and t+90)
    acc = {}
    for l in lines:
        x1, y1, x2, y2 = l[0]
        a = math.atan2(y2 - y1, x2 - x1) % (math.pi / 2)
        length = math.hypot(x2 - x1, y2 - y1)
        k = round(math.degrees(a) * 2) / 2.0          # 0.5 deg bins
        acc[k] = acc.get(k, 0.0) + length
    best = max(acc.items(), key=lambda kv: kv[1])[0]
    print(f"  dominant wall angle: {best:.1f} deg  "
          f"(from {len(lines)} segments)")
    return math.radians(best)


# ------------------------------------------------- BCD core
def decompose_cells(mask):
    """
    Boustrophedon Cellular Decomposition.
    mask: bool array, True = drivable (already eroded by robot radius).
    Sweeps left->right one COLUMN at a time. Where the count of connected
    vertical slices changes, that is a critical point -> new cells.

    Returns: list of cells; each cell is a list of (col, r_start, r_end).
    """
    h, w = mask.shape
    cells = []          # finished
    active = []         # list of dicts: {'slices':[(r0,r1)], 'data':[...]}

    def slices_of(col):
        """connected runs of True in this column"""
        colv = mask[:, col]
        out, start = [], None
        for r in range(h):
            if colv[r] and start is None:
                start = r
            elif not colv[r] and start is not None:
                out.append((start, r - 1)); start = None
        if start is not None:
            out.append((start, h - 1))
        return out

    prev = []
    prev_cells = []     # parallel to prev: which active cell owns each slice

    for col in range(w):
        cur = slices_of(col)
        cur_cells = [None] * len(cur)

        # connectivity: does cur[j] overlap prev[i]?
        for j, (a0, a1) in enumerate(cur):
            owners = []
            for i, (b0, b1) in enumerate(prev):
                if not (a1 < b0 or a0 > b1):        # overlap
                    owners.append(i)
            if len(owners) == 1:
                # count how many cur slices claim this same prev slice
                claimers = [k for k, (c0, c1) in enumerate(cur)
                            if not (c1 < prev[owners[0]][0] or c0 > prev[owners[0]][1])]
                if len(claimers) == 1:
                    # simple continuation
                    cur_cells[j] = prev_cells[owners[0]]
                else:
                    # IN event: prev slice splits -> close it, open new cells
                    old = prev_cells[owners[0]]
                    if old is not None and old in active:
                        cells.append(old['data']); active.remove(old)
                    cur_cells[j] = None
            elif len(owners) == 0:
                cur_cells[j] = None                  # brand new region
            else:
                # OUT event: several prev slices merge -> close them all
                for i in owners:
                    old = prev_cells[i]
                    if old is not None and old in active:
                        cells.append(old['data']); active.remove(old)
                cur_cells[j] = None

        # open cells for anything unassigned
        for j in range(len(cur)):
            if cur_cells[j] is None:
                c = {'data': []}
                active.append(c)
                cur_cells[j] = c

        for j, (r0, r1) in enumerate(cur):
            cur_cells[j]['data'].append((col, r0, r1))

        # anything in prev that has no overlap in cur is finished
        for i, (b0, b1) in enumerate(prev):
            alive = any(not (a1 < b0 or a0 > b1) for (a0, a1) in cur)
            if not alive:
                old = prev_cells[i]
                if old is not None and old in active:
                    cells.append(old['data']); active.remove(old)

        prev, prev_cells = cur, cur_cells

    for c in active:
        cells.append(c['data'])
    return [c for c in cells if len(c) > 0]


def serpentine(cell, lane_px, start_top=True):
    """Boustrophedon inside ONE cell: vertical lanes, alternating direction."""
    if not cell:
        return []
    cols = sorted(set(c for c, _, _ in cell))
    bycol = {}
    for c, r0, r1 in cell:
        bycol.setdefault(c, []).append((r0, r1))

    pts, up = [], start_top
    c = cols[0]
    while c <= cols[-1]:
        if c in bycol:
            r0 = min(a for a, _ in bycol[c])
            r1 = max(b for _, b in bycol[c])
            pts.append((c, r1 if up else r0))
            pts.append((c, r0 if up else r1))
            up = not up
        c += lane_px
    # always finish the last column of the cell
    if cols[-1] not in [p[0] for p in pts[-2:]] and cols[-1] in bycol:
        r0 = min(a for a, _ in bycol[cols[-1]])
        r1 = max(b for _, b in bycol[cols[-1]])
        pts.append((cols[-1], r1 if up else r0))
        pts.append((cols[-1], r0 if up else r1))
    return pts


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--start', nargs=2, type=float, required=True,
                    metavar=('X', 'Y'))
    ap.add_argument('--robot-radius', type=float, default=0.210)
    ap.add_argument('--safety-margin', type=float, default=0.10,
                    help='clearance BEYOND robot_radius. 0.10 keeps lanes well '
                         'off the walls (total erosion 0.31 m)')
    ap.add_argument('--lane-spacing', type=float, default=0.28,
                    help='0.28 = 2 cm overlap on a 0.30 m head. 0.19 was a '
                         '37%% double-clean and near 2x the runtime.')
    ap.add_argument('--start-corner', choices=['tl','tr','bl','br'], default='tl',
                    help='which corner of the room to begin sweeping from')
    ap.add_argument('--waypoint-spacing', type=float, default=0.15)
    ap.add_argument('--align', choices=['walls', 'perpendicular'], default='walls')
    ap.add_argument('--sweep-angle', type=float, default=None,
                    help='force lane angle in degrees, overrides --align')
    ap.add_argument('--seal-gaps', type=float, default=0.06)
    ap.add_argument('--v-max', type=float, default=0.0704)
    args = ap.parse_args()

    print(f"\n=== BCD coverage :: {args.map} ===")
    free, occ, unknown, res, origin, raw = load_map(args.map)
    h, w = free.shape
    print(f"  map {w}x{h} @ {res} m/px   origin {origin[:2]}")

    # seal hairline gaps so coverage cannot leak through a wall
    if args.seal_gaps > 0:
        k = max(1, int(round(args.seal_gaps / res)))
        occ = cv2.morphologyEx(occ.astype(np.uint8), cv2.MORPH_CLOSE,
                               np.ones((k, k), np.uint8)).astype(bool)

    # unknown counts as blocked -- never plan into it
    blocked = occ | unknown

    # erode free space by robot radius -> where the CENTRE may go
    er = int(math.ceil((args.robot_radius + args.safety_margin) / res))
    drive = cv2.erode((~blocked).astype(np.uint8),
                      cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                (2 * er + 1, 2 * er + 1))).astype(bool)
    print(f"  eroded by {args.robot_radius + args.safety_margin:.3f} m ({er} px)")

    # keep only the component the robot can actually reach from --start
    sr, sc = world_to_px(args.start[0], args.start[1], origin, res, h)
    if not (0 <= sr < h and 0 <= sc < w):
        sys.exit(f"start {args.start} is outside the map")
    lab, n = ndimage.label(drive)
    if not drive[sr, sc]:
        # nudge to nearest drivable pixel
        d, idx = ndimage.distance_transform_edt(~drive, return_indices=True)
        sr, sc = idx[0][sr, sc], idx[1][sr, sc]
        print(f"  ! start was not drivable, snapped to px ({sr},{sc})")
    reach = (lab == lab[sr, sc])
    print(f"  reachable area: {reach.sum() * res * res:.2f} m^2 "
          f"({n} components, kept 1)")

    # ---- lane angle
    if args.sweep_angle is not None:
        ang = math.radians(args.sweep_angle)
        print(f"  sweep angle FORCED to {args.sweep_angle:.1f} deg")
    else:
        ang = dominant_wall_angle(occ, res)
        if args.align == 'perpendicular':
            ang += math.pi / 2
            print(f"  --align perpendicular -> {math.degrees(ang):.1f} deg")

    # ---- rotate so lanes become vertical columns
    cx, cy = w / 2.0, h / 2.0
    M = cv2.getRotationMatrix2D((cx, cy), math.degrees(ang), 1.0)
    diag = int(math.ceil(math.hypot(w, h)))
    M[0, 2] += (diag - w) / 2.0
    M[1, 2] += (diag - h) / 2.0
    rot = cv2.warpAffine(reach.astype(np.uint8), M, (diag, diag),
                         flags=cv2.INTER_NEAREST).astype(bool)

    # ---- BCD
    cells = decompose_cells(rot)
    print(f"  cellular decomposition -> {len(cells)} cells")

    lane_px = max(1, int(round(args.lane_spacing / res)))
    Minv = cv2.invertAffineTransform(M)

    # ---- build all 4 corner variants, keep the one that starts nearest the
    #      requested world corner. "top-left" is meaningless in the ROTATED
    #      frame, so we decide it in WORLD coords after un-rotating.
    ys, xs = np.where(reach)
    wx = [px_to_world(r, c, origin, res, h)[0] for r, c in zip(ys[::37], xs[::37])]
    wy = [px_to_world(r, c, origin, res, h)[1] for r, c in zip(ys[::37], xs[::37])]
    xmin, xmax, ymin, ymax = min(wx), max(wx), min(wy), max(wy)
    want = {'tl': (xmin, ymax), 'tr': (xmax, ymax),
            'bl': (xmin, ymin), 'br': (xmax, ymin)}[args.start_corner]

    def build(rev_cells, first_up):
        cs = sorted(cells, key=lambda c: min(x for x, _, _ in c), reverse=rev_cells)
        out, up = [], first_up
        for cell in cs:
            area = sum((r1 - r0 + 1) for _, r0, r1 in cell) * res * res
            if area < 0.15:
                continue
            sp = serpentine(cell, lane_px, start_top=up)
            if not sp:
                continue
            up = not up
            for (c, r) in sp:
                v = Minv @ np.array([c, r, 1.0])
                rr, cc = int(round(v[1])), int(round(v[0]))
                if 0 <= rr < h and 0 <= cc < w:
                    out.append(px_to_world(rr, cc, origin, res, h))
        return out

    best, bestd = None, 1e18
    for rc in (False, True):
        for fu in (False, True):
            cand = build(rc, fu)
            if len(cand) < 2:
                continue
            d = math.hypot(cand[0][0] - want[0], cand[0][1] - want[1])
            if d < bestd:
                bestd, best = d, cand
    pts_world = best if best else []
    print(f"  start corner : {args.start_corner} -> first waypoint "
          f"{bestd:.2f} m from the {args.start_corner} corner of the room")

    if len(pts_world) < 2:
        sys.exit("no path produced -- is the map right? is --start inside it?")

    # ---- densify + yaw
    wp = args.waypoint_spacing
    poses = []
    for i in range(len(pts_world) - 1):
        x0, y0 = pts_world[i]; x1, y1 = pts_world[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(round(seg / wp)))
        for k in range(n):
            t = k / n
            poses.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw))
    poses.append((pts_world[-1][0], pts_world[-1][1], poses[-1][2]))

    length = sum(math.hypot(poses[i+1][0]-poses[i][0], poses[i+1][1]-poses[i][1])
                 for i in range(len(poses)-1))
    minutes = (length / args.v_max) / 60.0 * 1.35     # 35% for turns/accel

    # ---- write yaml (SAME schema coverage_server.py already reads)
    base = os.path.splitext(os.path.abspath(args.map))[0]
    out = {
        'metadata': {
            'generator': 'bcd_coverage.py (Boustrophedon Cellular Decomposition)',
            'generated': datetime.datetime.now().isoformat(timespec='seconds'),
            'map': os.path.basename(args.map),
            'sweep_deg': round(math.degrees(ang), 2),
            'align': args.align if args.sweep_angle is None else 'forced',
            'lane_spacing_m': args.lane_spacing,
            'wall_clearance_m': round(args.robot_radius + args.safety_margin, 3),
            'start_corner': args.start_corner,
            'n_cells': len(cells),
            'n_edge_waypoints': 0,          # <-- NO EDGE PASS. lanes from step 1.
            'n_waypoints': len(poses),
            'path_length_m': round(length, 2),
            'estimated_minutes': round(minutes, 1),
            'coverage_pct': 0.0,
        },
        'poses': [{'x': round(float(x), 4),
                   'y': round(float(y), 4),
                   'yaw': round(float(t), 4)} for (x, y, t) in poses],
    }
    yp = base + '_coverage_path.yaml'
    with open(yp, 'w') as f:
        yaml.safe_dump(out, f, sort_keys=False)

    # ---- preview
    vis = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
    vis[reach] = (0.6 * vis[reach] + np.array([60, 0, 0])).astype(np.uint8)
    for i in range(len(poses) - 1):
        r0, c0 = world_to_px(poses[i][0], poses[i][1], origin, res, h)
        r1, c1 = world_to_px(poses[i+1][0], poses[i+1][1], origin, res, h)
        cv2.line(vis, (c0, r0), (c1, r1), (0, 0, 255), 1)
    r, c = world_to_px(poses[0][0], poses[0][1], origin, res, h)
    cv2.circle(vis, (c, r), 4, (0, 255, 0), -1)
    r, c = world_to_px(poses[-1][0], poses[-1][1], origin, res, h)
    cv2.circle(vis, (c, r), 4, (255, 0, 255), -1)
    pp = base + '_coverage_preview.png'
    cv2.imwrite(pp, vis)

    print(f"\n  cells        : {len(cells)}")
    print(f"  waypoints    : {len(poses)}")
    print(f"  path length  : {length:.1f} m")
    print(f"  est. runtime : {minutes:.1f} min  @ {args.v_max} m/s")
    print(f"\n  -> {yp}")
    print(f"  -> {pp}   <-- LOOK AT THIS BEFORE YOU RUN\n")


if __name__ == '__main__':
    main()
