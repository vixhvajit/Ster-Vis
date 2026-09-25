"""Ster-Vis test AMR, minimal version: one printed deck, glue for the rest.

A temporary robot for trying Ster-Vis on real hardware, built to be printed and
running in an afternoon. The chassis is one PLA part: a 3 mm deck with the
camera bracket printed in one piece with it. Apart from the camera clamp there
are no fasteners:

- the four BO motors are glued into shallow pockets on the underside, which
  line them up (epoxy, or plenty of hot glue);
- the Pi 5 and the two L298Ns drop onto posts whose pegs go through their
  mounting holes; a dab of hot glue on each peg holds them;
- the battery (Bonka 3S 2200 mAh, 105 x 34 x 25 mm) stands on its long edge in
  a tray, and lifts out for charging; its leads leave through a notch;
- the buck converter sits in a low rim, with a dab of glue;
- each camera slides down between rails onto a ledge, and a printed cover,
  pressed by two tapered keys, clamps it against four stops. Nothing touches
  the board except around its four holes, and it can come out again.

Two L298Ns, one per side, as the Ster-Vis AMR software expects.

Builds the robot in a new Fusion 360 design with stand-ins for the bought
parts, checks for interferences, and exports print-ready STLs to ./stl, the
.f3d to ./cad, and the printed parts where they sit on the robot to
./cad/assembly (for the Gazebo model).

Run it inside Fusion: Utilities > Scripts and Add-Ins, or through the fusion360
MCP bridge (exec this file with __file__ set). All numbers are millimetres.
Frame: x forward, y left, z up. z = 0 is the underside of the deck.
"""
import math
import os

import adsk.core
import adsk.fusion

# ---- TT (BO) gear motor, straight type ---------------------------------------
TT_THICK = 18.8         # along the shaft
TT_HEIGHT = 22.5
TT_FRONT = 11.9         # shaft axis to the gearbox's front end
TT_GEARBOX = 48.0       # front end to where the motor can starts
TT_LENGTH = 70.0        # front end to the back of the motor can
TT_SHAFT_D, TT_SHAFT_OUT = 5.4, 8.7
WHEEL_D, WHEEL_W, WHEEL_GAP = 65.0, 26.0, 2.0

# ---- deck ----------------------------------------------------------------------
DECK_X0, DECK_X1 = -75.0, 82.0   # the front reaches under the camera rails, so they print from it
DECK_W, DECK_T, CORNER_R = 136.0, 3.0, 6.0
MOTOR_X = 62.0                   # shaft x of the front motors; wheelbase is twice this
MOTOR_OUTER_Y = DECK_W / 2       # gearbox outer face, flush with the deck's side
POCKET = 0.8                     # motor pocket depth in the underside; the gearbox top glues in here
POCKET_CLEAR = 0.3
Z_TOP = DECK_T
SHAFT_Z = POCKET - TT_HEIGHT / 2             # the motor's top sits in the pocket
FLOOR_Z = SHAFT_Z - WHEEL_D / 2              # the floor, in this frame
# motor wires up to the L298Ns: just inside each side's motors, behind the Pi
WIRE_SLOTS = [(-12.0, -4.0, 44.5, 48.5), (-12.0, -4.0, -48.5, -44.5)]

# ---- battery tray: Bonka 3S 2200 mAh 35C, 105 x 34 x 25 mm, standing on its long edge ----
BATTERY = (-72.5, -47.5, 52.5, 34.0)       # x from, x to (25 thick), half-length along y, height
TRAY_WALL, TRAY_H, TRAY_CLEAR = 2.0, 20.0, 0.5
LEAD_NOTCH = 16.0                            # in the +y end wall, for the XT60 and balance leads

# ---- two L298Ns on posts, left and right --------------------------------------------
L298N_CENTRES = [(-23.0, 22.5), (-23.0, -22.5)]
L298N_BOARD, L298N_HOLE = 43.0, 18.5         # 37 mm square hole pattern
L298N_POST_D, L298N_POST_H, L298N_PEG_D = 6.0, 5.0, 2.6

# ---- Pi 5 on posts, turned so its long side runs across the robot ------------------
PI_X = 29.0                      # board centre x
PI_SD_Y = 42.5                   # the SD-card edge; USB and Ethernet face the right (-y)
PI_POST_D, PI_POST_H, PI_PEG_D = 6.0, 4.0, 2.4
PEG_LEN = 2.4                    # peg above a post: through a 1.6 mm board, with room for glue

# ---- buck converter in a rim, beside the Pi -------------------------------------------
BUCK = (1.0, 55.0, 43.0, 66.0)               # x0, x1, y0, y1 (5 V 5 A module stand-in)
BUCK_H, RIM_W, RIM_H = 20.0, 1.5, 3.0

# ---- camera bracket, part of the deck ------------------------------------------------
BASELINE = 60.0
PLATE_X, PLATE_T, PLATE_HALF_W = 68.0, 4.0, 46.0
CAM_BOARD_W, CAM_BOARD_H, CAM_BOARD_T = 25.0, 24.0, 1.1
CAM_BOARD_Z0 = Z_TOP + 8.0                   # room under the board for the ribbon bend
CAM_HOLE_DX, CAM_HOLE_DY, CAM_HOLE_TOP = 21.0, 12.5, 2.0
CAM_STOP_D, CAM_STOP_H = 4.5, 3.0            # rear stops behind the board's four holes
GUSSET_X0, GUSSET_T = 58.0, 4.0
# Rails either side of each board, a ledge under it, a cover in front of it
# (touching only round the four holes), and two tapered keys between the cover
# and the rails' lips.
X_BOARD = PLATE_X + PLATE_T + CAM_STOP_H     # back of the camera board
X_BOARD_FRONT = X_BOARD + CAM_BOARD_T
PAD_H, PAD_D, COVER_T = 1.0, 4.0, 1.2        # the cover's pads clear anything on the board's front
KEY_GAP = 1.3                                # key thickness where it wedges
X_LIP0 = X_BOARD_FRONT + PAD_H + COVER_T + KEY_GAP
X_LIP1 = X_LIP0 + 1.2
RAIL_IN, RAIL_T = CAM_BOARD_W / 2 + 0.15, 1.6
LIP_IN = 10.15                               # lips reach in this far from the camera's centre line
LEDGE_IN = 9.5                               # the 16 mm ribbon passes between the ledges
RAIL_TOP = CAM_BOARD_Z0 + CAM_BOARD_H + 2.0
PLATE_TOP = RAIL_TOP + 1.0
KEY_W, KEY_L, KEY_T0 = 2.5, 32.0, 0.8
KEY_SLOPE = (KEY_GAP - KEY_T0) / 18.0        # wedges 8 mm above the ledge, so it has room either way
COVER_WINDOW = (12.0, 13.0)                  # around the lens

MM = 0.1  # Fusion works in cm


def P(x, y, z):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def V(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


class Builder:
    def __init__(self):
        self.tbm = adsk.fusion.TemporaryBRepManager.get()
        self.UNION = adsk.fusion.BooleanTypes.UnionBooleanType
        self.CUT = adsk.fusion.BooleanTypes.DifferenceBooleanType
        self.INTERSECT = adsk.fusion.BooleanTypes.IntersectionBooleanType

    def box(self, x0, x1, y0, y1, z0, z1):
        x0, x1 = min(x0, x1), max(x0, x1)
        y0, y1 = min(y0, y1), max(y0, y1)
        z0, z1 = min(z0, z1), max(z0, z1)
        obb = adsk.core.OrientedBoundingBox3D.create(
            P((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), V(1, 0, 0), V(0, 1, 0),
            (x1 - x0) * MM, (y1 - y0) * MM, (z1 - z0) * MM)
        return self.tbm.createBox(obb)

    def cyl(self, p0, p1, d):
        return self.tbm.createCylinderOrCone(P(*p0), d / 2 * MM, P(*p1), d / 2 * MM)

    def zcyl(self, x, y, z0, z1, d):
        return self.cyl((x, y, z0), (x, y, z1), d)

    def ycyl(self, x, z, y0, y1, d):
        return self.cyl((x, y0, z), (x, y1, z), d)

    def xcyl(self, y, z, x0, x1, d):
        return self.cyl((x0, y, z), (x1, y, z), d)

    def union(self, target, *tools):
        for tool in tools:
            if not self.tbm.booleanOperation(target, tool, self.UNION):
                raise RuntimeError("union failed")
        return target

    def cut(self, target, *tools):
        for tool in tools:
            if not self.tbm.booleanOperation(target, tool, self.CUT):
                raise RuntimeError("cut failed")
        return target

    def moved(self, body, dx=0.0, dy=0.0, dz=0.0, rot=None):
        """A copy, rotated first (rot = (angle_rad, axis vector)) about the origin, then translated."""
        c = self.tbm.copy(body)
        if rot is not None:
            m = adsk.core.Matrix3D.create()
            m.setToRotation(rot[0], V(*rot[1]), P(0, 0, 0))
            self.tbm.transform(c, m)
        m = adsk.core.Matrix3D.create()
        m.translation = V(dx * MM, dy * MM, dz * MM)
        self.tbm.transform(c, m)
        return c

    def rounded_plate(self, x0, x1, y0, y1, z0, z1, r):
        plate = self.box(x0, x1, y0, y1, z0, z1)
        for cx, sx in ((x0, 1), (x1, -1)):
            for cy, sy in ((y0, 1), (y1, -1)):
                self.cut(plate, self.box(cx, cx + sx * r, cy, cy + sy * r, z0 - 1, z1 + 1))
                self.union(plate, self.zcyl(cx + sx * r, cy + sy * r, z0, z1, 2 * r))
        return plate

    def cut_beyond(self, body, a, b, y0, y1, keep):
        """Cut away everything on the far side of the line a-b (in the xz plane,
        extruded along y) from the point ``keep``."""
        (ax, az), (bx, bz) = a, b
        ux, uz = bx - ax, bz - az
        n = math.hypot(ux, uz)
        ux, uz = ux / n, uz / n
        nx, nz = uz, -ux
        mx, mz = (ax + bx) / 2, (az + bz) / 2
        if (keep[0] - mx) * nx + (keep[1] - mz) * nz > 0:
            nx, nz = -nx, -nz
        s = 400.0
        obb = adsk.core.OrientedBoundingBox3D.create(
            P(mx + nx * s / 2, (y0 + y1) / 2, mz + nz * s / 2), V(ux, 0, uz), V(nx, 0, nz),
            s * MM, s * MM, (abs(y1 - y0) + 2) * MM)
        return self.cut(body, self.tbm.createBox(obb))

    def post(self, x, y, d, h, peg_d):
        p = self.zcyl(x, y, Z_TOP - 0.5, Z_TOP + h, d)
        return self.union(p, self.zcyl(x, y, Z_TOP + h - 0.1, Z_TOP + h + PEG_LEN, peg_d))

    def open_box(self, x0, x1, y0, y1, wall, height):
        """Walls round the rectangle x0..x1, y0..y1 (the inside), standing on the deck."""
        outer = self.box(x0 - wall, x1 + wall, y0 - wall, y1 + wall, Z_TOP - 0.5, Z_TOP + height)
        return self.cut(outer, self.box(x0, x1, y0, y1, Z_TOP, Z_TOP + height + 1))

    # -- the printed parts -------------------------------------------------------
    def deck(self):
        deck = self.rounded_plate(DECK_X0, DECK_X1, -DECK_W / 2, DECK_W / 2, 0, DECK_T, CORNER_R)
        # motor pockets in the underside, open to the side
        inner = MOTOR_OUTER_Y - TT_THICK
        for s in (1, -1):
            for t in (1, -1):
                sx = s * MOTOR_X
                self.cut(deck, self.box(sx + s * (TT_FRONT + POCKET_CLEAR),
                                        sx + s * (TT_FRONT - TT_GEARBOX - POCKET_CLEAR),
                                        t * (inner - POCKET_CLEAR), t * (MOTOR_OUTER_Y + 1), -1, POCKET))
        for x0, x1, y0, y1 in WIRE_SLOTS:
            self.cut(deck, self.box(x0, x1, y0, y1, -1, DECK_T + 1))
        # battery tray, with a notch for the leads and finger slots to lift it out
        bx0, bx1, bhl, _ = BATTERY
        c = TRAY_CLEAR
        tray = self.open_box(bx0 - c, bx1 + c, -bhl - c, bhl + c, TRAY_WALL, TRAY_H)
        self.cut(tray, self.box(bx0 - c, bx1 + c, bhl, bhl + c + TRAY_WALL + 1, Z_TOP + 3, Z_TOP + TRAY_H + 1))
        for y in (-25.0, 25.0):
            self.cut(tray, self.box(bx0 - c - TRAY_WALL - 1, bx1 + c + TRAY_WALL + 1, y - 8, y + 8,
                                    Z_TOP + 8, Z_TOP + TRAY_H + 1))
        self.union(deck, tray)
        # buck converter rim
        ux0, ux1, uy0, uy1 = BUCK
        self.union(deck, self.open_box(ux0 - c, ux1 + c, uy0 - c, uy1 + c, RIM_W, RIM_H))
        # posts
        for cx, cy in L298N_CENTRES:
            for dx in (-L298N_HOLE, L298N_HOLE):
                for dy in (-L298N_HOLE, L298N_HOLE):
                    self.union(deck, self.post(cx + dx, cy + dy, L298N_POST_D, L298N_POST_H, L298N_PEG_D))
        for hx, hy in pi_holes():
            self.union(deck, self.post(hx, hy, PI_POST_D, PI_POST_H, PI_PEG_D))
        # camera bracket: plate, gussets, stops, windows, rails, lips and ledges
        front = PLATE_X + PLATE_T
        self.union(deck, self.box(PLATE_X, front, -PLATE_HALF_W, PLATE_HALF_W, Z_TOP - 0.5, PLATE_TOP))
        for y0 in (-GUSSET_T / 2, PLATE_HALF_W - GUSSET_T, -PLATE_HALF_W):
            g = self.box(GUSSET_X0, PLATE_X + 0.5, y0, y0 + GUSSET_T, Z_TOP - 0.5, PLATE_TOP - 2)
            self.union(deck, self.cut_beyond(g, (GUSSET_X0, Z_TOP - 0.5), (PLATE_X + 0.5, PLATE_TOP - 2),
                                             y0, y0 + GUSSET_T, (PLATE_X, Z_TOP)))
        for cy in (BASELINE / 2, -BASELINE / 2):
            for hy, hz in cam_holes(cy):
                self.union(deck, self.xcyl(hy, hz, front - 0.5, X_BOARD, CAM_STOP_D))
            top_hole_z = CAM_BOARD_Z0 + CAM_BOARD_H - CAM_HOLE_TOP
            # window for the parts on the back of the board, widened low down for the 16 mm ribbon
            self.cut(deck, self.box(PLATE_X - 1, front + 1, cy - 7, cy + 7, Z_TOP, top_hole_z - 1))
            self.cut(deck, self.box(PLATE_X - 1, front + 1, cy - 9.5, cy + 9.5, Z_TOP,
                                    top_hole_z - CAM_HOLE_DY - CAM_STOP_D / 2 - 0.5))
        for cy in (BASELINE / 2, -BASELINE / 2):
            for k in (1, -1):
                # all stand on the deck's front, so they print without support
                self.union(deck, self.box(front - 0.5, X_LIP1, cy + k * RAIL_IN, cy + k * (RAIL_IN + RAIL_T),
                                          Z_TOP - 0.5, RAIL_TOP))
                self.union(deck, self.box(X_LIP0, X_LIP1, cy + k * LIP_IN, cy + k * (RAIL_IN + 0.1),
                                          Z_TOP - 0.5, RAIL_TOP))
                self.union(deck, self.box(front - 0.5, X_LIP0, cy + k * LEDGE_IN, cy + k * (RAIL_IN + 0.1),
                                          Z_TOP - 0.5, CAM_BOARD_Z0))
        return deck

    def camera_cover(self, cy):
        body = self.box(X_BOARD_FRONT + PAD_H, X_BOARD_FRONT + PAD_H + COVER_T, cy - CAM_BOARD_W / 2,
                        cy + CAM_BOARD_W / 2, CAM_BOARD_Z0, RAIL_TOP)
        for hy, hz in cam_holes(cy):
            self.union(body, self.xcyl(hy, hz, X_BOARD_FRONT, X_BOARD_FRONT + PAD_H + 0.5, PAD_D))
        wy, wz = COVER_WINDOW
        lz = lens_z()
        return self.cut(body, self.box(X_BOARD_FRONT - 1, X_BOARD_FRONT + 5, cy - wy / 2, cy + wy / 2,
                                       lz - wz / 2, lz + wz / 2))

    def camera_key(self, cy, k):
        """Tapered key between the cover and one rail's lip, wedged home."""
        x0 = X_BOARD_FRONT + PAD_H + COVER_T
        z0 = RAIL_TOP - (KEY_GAP - 0.01 - KEY_T0) / KEY_SLOPE
        y0, y1 = cy + k * (LIP_IN - 0.15), cy + k * (LIP_IN - 0.15 + KEY_W)
        t1 = KEY_T0 + KEY_L * KEY_SLOPE
        key = self.box(x0, x0 + t1, y0, y1, z0, z0 + KEY_L)
        return self.cut_beyond(key, (x0 + KEY_T0, z0), (x0 + t1, z0 + KEY_L), y0, y1, (x0, z0 + KEY_L / 2))

    # -- stand-ins for bought parts -------------------------------------------
    def motor(self, s, t):
        sx = s * MOTOR_X
        inner = MOTOR_OUTER_Y - TT_THICK
        m = self.box(sx + s * TT_FRONT, sx + s * (TT_FRONT - TT_GEARBOX), t * inner, t * MOTOR_OUTER_Y,
                     POCKET - TT_HEIGHT, POCKET)
        can = self.box(sx + s * (TT_FRONT - TT_GEARBOX), sx + s * (TT_FRONT - TT_LENGTH),
                       t * (inner + 1.9), t * (MOTOR_OUTER_Y - 1.9), SHAFT_Z - 10, SHAFT_Z + 10)
        shaft = self.ycyl(sx, SHAFT_Z, t * (inner - TT_SHAFT_OUT), t * (MOTOR_OUTER_Y + TT_SHAFT_OUT), TT_SHAFT_D)
        return self.union(m, can, shaft)

    def wheel(self, s, t):
        y0 = MOTOR_OUTER_Y + WHEEL_GAP
        return self.ycyl(s * MOTOR_X, SHAFT_Z, t * y0, t * (y0 + WHEEL_W), WHEEL_D)

    def board_with_holes(self, x0, x1, y0, y1, z0, holes, hole_d):
        board = self.box(x0, x1, y0, y1, z0, z0 + 1.6)
        for hx, hy in holes:
            self.cut(board, self.zcyl(hx, hy, z0 - 1, z0 + 3, hole_d))
        return board

    def pi5(self):
        z0 = Z_TOP + PI_POST_H
        y_ports = PI_SD_Y - 85           # USB and Ethernet end
        pi = self.board_with_holes(PI_X - 28, PI_X + 28, y_ports, PI_SD_Y, z0, pi_holes(), 2.7)
        ports = self.box(PI_X - 26, PI_X + 26, y_ports - 2, y_ports + 22, z0 + 1.6, z0 + 17.6)
        cooler = self.box(PI_X - 22, PI_X + 22, y_ports + 26, PI_SD_Y - 8, z0 + 1.6, z0 + 32)
        return self.union(pi, ports, cooler)

    def l298n(self, cx, cy):
        z0 = Z_TOP + L298N_POST_H
        h = L298N_BOARD / 2
        holes = [(cx + dx, cy + dy) for dx in (-L298N_HOLE, L298N_HOLE) for dy in (-L298N_HOLE, L298N_HOLE)]
        board = self.board_with_holes(cx - h, cx + h, cy - h, cy + h, z0, holes, 3.0)
        return self.union(board, self.box(cx - 11.5, cx + 11.5, cy - 8, cy + 8, z0 + 1.6, z0 + 27))

    def camera(self, cy):
        board = self.box(X_BOARD, X_BOARD_FRONT, cy - CAM_BOARD_W / 2, cy + CAM_BOARD_W / 2,
                         CAM_BOARD_Z0, CAM_BOARD_Z0 + CAM_BOARD_H)
        lz = lens_z()
        lens = self.box(X_BOARD_FRONT, X_BOARD_FRONT + 5, cy - 4.25, cy + 4.25, lz - 4.25, lz + 4.25)
        return self.union(board, lens)


def pi_holes():
    ys = (PI_SD_Y - 3.5, PI_SD_Y - 61.5)
    return [(PI_X + dx, y) for y in ys for dx in (24.5, -24.5)]


def cam_holes(cy):
    top = CAM_BOARD_Z0 + CAM_BOARD_H - CAM_HOLE_TOP
    return [(cy + dy, z) for dy in (-CAM_HOLE_DX / 2, CAM_HOLE_DX / 2) for z in (top, top - CAM_HOLE_DY)]


def lens_z():
    """Stand-in lens height: midway between the hole rows. Measure the real one."""
    top = CAM_BOARD_Z0 + CAM_BOARD_H - CAM_HOLE_TOP
    return top - CAM_HOLE_DY / 2


def build(app, out_dir):
    b = Builder()
    tbm = b.tbm
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    root = design.rootComponent

    def new_comp(name):
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = name
        return occ

    deck = b.deck()
    clamp = {}
    for cy, side in ((BASELINE / 2, "left"), (-BASELINE / 2, "right")):
        clamp["Camera cover %s" % side] = b.camera_cover(cy)
        for k in (1, -1):
            clamp["Camera key %s %s" % (side, "outer" if k * cy > 0 else "inner")] = b.camera_key(cy, k)
    ref = {}
    for s, sname in ((1, "front"), (-1, "rear")):
        for t, tname in ((1, "left"), (-1, "right")):
            ref["Motor %s %s" % (sname, tname)] = b.motor(s, t)
            ref["Wheel %s %s" % (sname, tname)] = b.wheel(s, t)
    ref["Pi 5 + Active Cooler"] = b.pi5()
    for cx, cy in L298N_CENTRES:
        ref["L298N %s" % ("left" if cy > 0 else "right")] = b.l298n(cx, cy)
    bx0, bx1, bhl, bh = BATTERY
    ref["Bonka 3S 2200 mAh battery"] = b.box(bx0, bx1, -bhl, bhl, Z_TOP, Z_TOP + bh)
    ux0, ux1, uy0, uy1 = BUCK
    ref["5 V buck converter"] = b.box(ux0, ux1, uy0, uy1, Z_TOP, Z_TOP + BUCK_H)
    ref["Camera left"] = b.camera(BASELINE / 2)
    ref["Camera right"] = b.camera(-BASELINE / 2)

    def overlap(ba, bb):
        if not ba.boundingBox.intersects(bb.boundingBox):
            return 0.0
        probe = tbm.copy(ba)
        try:
            ok = tbm.booleanOperation(probe, tbm.copy(bb), b.INTERSECT)
            return probe.volume * 1000 if ok else 0.0
        except Exception:
            return 0.0

    everything = [("Deck (PLA)", deck)] + list(clamp.items()) + list(ref.items())
    clashes = []
    for i in range(len(everything)):
        for j in range(i + 1, len(everything)):
            na, ba = everything[i]
            nb, bb = everything[j]
            if na.startswith("Motor") and nb == na.replace("Motor", "Wheel"):
                continue    # the shaft sits in the wheel hub by design
            vol = overlap(ba, bb)
            if vol > 1.0:
                clashes.append((na, nb, round(vol, 1)))
    # the battery lifts straight out of its tray
    lift = b.box(bx0, bx1, -bhl, bhl, Z_TOP + 0.5, Z_TOP + 200)
    blocking = [n for n, body in everything if not n.startswith("Bonka") and overlap(lift, body) > 1.0]

    deck_occ = new_comp("Deck (PLA)")
    deck_occ.component.bRepBodies.add(deck).name = "Deck (PLA)"
    clamp_occ = new_comp("Camera clamps (PLA)")
    for name, body in clamp.items():
        clamp_occ.component.bRepBodies.add(body).name = name
    refs = new_comp("Bought parts (reference, not printed)").component
    for name, body in ref.items():
        refs.bRepBodies.add(body).name = name
    colour_bodies(app, design, refs)

    stl_dir = os.path.join(out_dir, "stl")
    cad_dir = os.path.join(out_dir, "cad")
    asm_dir = os.path.join(cad_dir, "assembly")
    for d in (stl_dir, cad_dir, asm_dir):
        os.makedirs(d, exist_ok=True)
    for folder in (stl_dir, asm_dir):
        for old in os.listdir(folder):
            if old.endswith(".stl"):
                os.remove(os.path.join(folder, old))
    em = design.exportManager
    for occ, fname in ((deck_occ, "deck"), (clamp_occ, "camera_clamps")):
        opts = em.createSTLExportOptions(occ, os.path.join(asm_dir, fname + ".stl"))
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        em.execute(opts)

    # print-oriented: the deck as it sits, bracket up; the cover front face down;
    # the key on its side, taper in the bed's plane
    to_print = {
        "deck_PLA": deck,
        "camera_cover_PLA_x2": b.moved(clamp["Camera cover left"], rot=(math.pi / 2, (0, 1, 0))),
        "camera_key_PLA_x4": b.moved(clamp["Camera key left outer"], rot=(math.pi / 2, (1, 0, 0))),
    }
    layout_occ = new_comp("Print layout (hidden)")
    exported = []
    next_x = 0.0
    for name, body in to_print.items():
        bb = body.boundingBox
        body = b.moved(body, next_x - bb.minPoint.x / MM, -(bb.minPoint.y + bb.maxPoint.y) / 2 / MM,
                       -bb.minPoint.z / MM)
        next_x += (bb.maxPoint.x - bb.minPoint.x) / MM + 15
        real = layout_occ.component.bRepBodies.add(body)
        real.name = name
        opts = em.createSTLExportOptions(real, os.path.join(stl_dir, name + ".stl"))
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        em.execute(opts)
        bb = real.boundingBox
        exported.append((name, [round((bb.maxPoint.x - bb.minPoint.x) * 10, 1),
                                round((bb.maxPoint.y - bb.minPoint.y) * 10, 1),
                                round((bb.maxPoint.z - bb.minPoint.z) * 10, 1)]))
    layout_occ.isLightBulbOn = False
    em.execute(em.createFusionArchiveExportOptions(os.path.join(cad_dir, "ster-vis-test-amr.f3d")))
    app.activeViewport.fit()

    return {
        "clashes": clashes,
        "battery_lift_blocked_by": blocking,
        "exported": exported,
        "deck_volume_cm3": round(deck.volume, 1),
        "lens_height_above_ground_mm": round(lens_z() - FLOOR_Z, 1),
        "lens_front_x_mm": X_BOARD_FRONT + 5,
        "deck_underside_above_ground_mm": round(-FLOOR_Z, 2),
        "ground_clearance_mm": round(POCKET - TT_HEIGHT - FLOOR_Z, 2),
        "track_mm": 2 * (MOTOR_OUTER_Y + WHEEL_GAP + WHEEL_W / 2),
    }


def colour_bodies(app, design, comp):
    """Best effort: colour the stand-ins so they read apart from the printed parts."""
    wanted = {"Motor": "Plastic - Glossy (Yellow)", "Wheel": "Rubber - Soft",
              "Pi": "Paint - Enamel Glossy (Green)", "L298N": "Plastic - Glossy (Red)",
              "Bonka": "Plastic - Glossy (Blue)", "5 V": "Paint - Enamel Glossy (Green)",
              "Camera": "Paint - Enamel Glossy (Green)"}
    lib = None
    for i in range(app.materialLibraries.count):
        if "Appearance" in app.materialLibraries.item(i).name:
            lib = app.materialLibraries.item(i)
            break
    if lib is None:
        return
    for body in comp.bRepBodies:
        for prefix, appearance in wanted.items():
            if body.name.startswith(prefix):
                try:
                    a = design.appearances.itemByName(appearance) or design.appearances.addByCopy(
                        lib.appearances.itemByName(appearance), appearance)
                    body.appearance = a
                except Exception:
                    pass


def run(context):
    app = adsk.core.Application.get()
    result = build(app, os.path.dirname(os.path.abspath(__file__)))
    print(result)
    return result


if __name__ == "__fusion_mcp__":
    run(None)
