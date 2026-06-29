#!/usr/bin/env python3
"""Generate deploy/pathplanner/navgrid.json from the field keep-outs.

PathPlanner owns the navgrid.json format (true = BLOCKED node, [row=y][col=x], node-size grid). The
rectangles below MIRROR frc.robot.autonomy.FieldGeometry — that Java class is the single source of
truth for legality (LegalRegion enforces it at runtime); this script just bakes the same keep-outs
into the static grid the pathfinder routes over, inflated by the robot clearance so cell centres stay
drivable. Re-run after changing FieldGeometry:  python3 tools/gen_navgrid.py

Keeps the alliance ZONES open (the old committed navgrid wrongly blocked most of the blue zone,
including the shoot/collect poses) while blocking the HUBS, TOWERS, and (too-tall-to-pass) TRENCHES.
"""
import json
import math
import os

FIELD_X = 16.541
FIELD_Y = 8.069
NODE = 0.2  # finer than the old 0.3 so robot-size inflation does not over-close the bump gaps

# PathPlanner pathfinds the robot as a POINT, so the navgrid must be inflated by the robot's size.
# Use the HALF-DIAGONAL (worst case at any heading) read from the PathPlanner robot config, so a
# point on an open cell is safe for the real chassis. This is what the PathPlanner GUI does with the
# robot dimensions; we replicate it here because we generate the grid ourselves.
_settings = json.load(
    open(os.path.join(os.path.dirname(__file__), "..", "src", "main", "deploy", "pathplanner", "settings.json"))
)
_robot_w = _settings.get("robotWidth", 0.838)
_robot_l = _settings.get("robotLength", 0.838)
CLEARANCE = math.hypot(_robot_w / 2.0, _robot_l / 2.0)  # ~0.592 m for a 0.838 m square

IN = 0.0254
hub_half = 47.0 * IN / 2.0
tower_w = 49.25 * IN
tower_d = 45.0 * IN
trench_w = 65.65 * IN

BLUE_HUB = (4.626, 4.035)
RED_HUB = (11.916, 4.035)
BLUE_TOWER_YC = 3.9616
RED_TOWER_YC = 4.1077


def centered(cx, cy, hx, hy):
    return (cx - hx, cy - hy, cx + hx, cy + hy)


# (minX, minY, maxX, maxY) — must match FieldGeometry.kKeepouts exactly.
KEEPOUTS = [
    centered(*BLUE_HUB, hub_half, hub_half),
    centered(*RED_HUB, hub_half, hub_half),
    (0.0, BLUE_TOWER_YC - tower_w / 2, tower_d, BLUE_TOWER_YC + tower_w / 2),
    (FIELD_X - tower_d, RED_TOWER_YC - tower_w / 2, FIELD_X, RED_TOWER_YC + tower_w / 2),
    (BLUE_HUB[0] - hub_half, 0.0, BLUE_HUB[0] + hub_half, trench_w),
    (BLUE_HUB[0] - hub_half, FIELD_Y - trench_w, BLUE_HUB[0] + hub_half, FIELD_Y),
    (RED_HUB[0] - hub_half, 0.0, RED_HUB[0] + hub_half, trench_w),
    (RED_HUB[0] - hub_half, FIELD_Y - trench_w, RED_HUB[0] + hub_half, FIELD_Y),
]


def blocked(x, y):
    # Off-field perimeter margin (keep the chassis off the walls).
    if x < CLEARANCE or x > FIELD_X - CLEARANCE or y < CLEARANCE or y > FIELD_Y - CLEARANCE:
        return True
    for (minx, miny, maxx, maxy) in KEEPOUTS:
        if (minx - CLEARANCE) <= x <= (maxx + CLEARANCE) and (miny - CLEARANCE) <= y <= (maxy + CLEARANCE):
            return True
    return False


cols = math.ceil(FIELD_X / NODE)  # x
rows = math.ceil(FIELD_Y / NODE)  # y
grid = [[blocked((c + 0.5) * NODE, (r + 0.5) * NODE) for c in range(cols)] for r in range(rows)]

out = {"field_size": {"x": FIELD_X, "y": FIELD_Y}, "nodeSizeMeters": NODE, "grid": grid}
path = os.path.join(os.path.dirname(__file__), "..", "src", "main", "deploy", "pathplanner", "navgrid.json")
with open(path, "w") as f:
    json.dump(out, f)

nblocked = sum(row.count(True) for row in grid)
print(f"wrote {path}: {rows}x{cols} grid, {nblocked} blocked / {rows*cols} cells")
