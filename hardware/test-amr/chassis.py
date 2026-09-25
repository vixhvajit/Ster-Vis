"""Ster-Vis test AMR: 3D-printed chassis, generated in Fusion 360.

Builds the whole robot in a new direct-modelling design: the printed parts, plus
stand-in models of the bought parts (motors, wheels, Pi 5, L298N drivers,
battery, buck converter, cameras) so fit can be checked. It then runs an
interference check and exports print-oriented STLs and an .f3d archive into
./stl and ./cad next to this file, and the printed parts in their assembled
positions into ./cad/assembly (for the Gazebo model).

No screws, nuts, straps or zip ties: every joint is printed. M2 and M3 threads
are too fine to print, so each joint uses something that prints well instead:

- decks and pillars: 6 mm pegs on the pillars, locked by tapered wedges
  pushed through a slot in each peg;
- motors: headed pins through the wall and the gearbox, kept in by the wheel;
- L298Ns: standoffs with snap pegs at both ends;
- Pi 5: snap pegs on the top deck's bosses;
- cameras: each board slides down between rails onto a ledge, and a cover
  clamps it against the rear stops, pressed by two tapered keys;
- battery and buck converter: sprung U-clips that snap through the deck.

Run it inside Fusion: Utilities > Scripts and Add-Ins, or through the fusion360
MCP bridge (exec this file with __file__ set).

All numbers are millimetres. The bought-part dimensions come from published
drawings and CAD models, not from your parts. TT (BO) motor clones vary by about
0.5 mm, so measure yours and edit the TT_* values before printing. The mounting
holes (TT_HOLE_BACK, TT_HOLE_SPACING) matter most.

Frame: x forward, y left, z up. z = 0 is the underside of the bottom deck.
"""
import math
import os

import adsk.core
import adsk.fusion

# ---- TT (BO) gear motor, straight type ---------------------------------------
TT_THICK = 18.8         # along the shaft
TT_HEIGHT = 22.5        # the two mounting holes are stacked across this
TT_FRONT = 11.9         # shaft axis to the gearbox's front end
TT_GEARBOX = 48.0       # front end to where the motor can starts
TT_LENGTH = 70.0        # front end to the back of the motor can, clip included
TT_HOLE_BACK = 20.2     # shaft axis to the mounting holes, along the motor
TT_HOLE_SPACING = 17.5  # between the two mounting holes
TT_HOLE_D = 3.2         # the gearbox's through holes
TT_PEG_BACK = 11.0      # locating peg on the gearbox face, behind the shaft
TT_SHAFT_D, TT_SHAFT_OUT = 5.4, 8.7
WHEEL_D, WHEEL_W, WHEEL_GAP = 65.0, 26.0, 2.0

# ---- bottom deck and motor walls ---------------------------------------------
DECK_L, DECK_W, DECK_T, CORNER_R = 150.0, 136.0, 4.0, 6.0
MOTOR_X = 62.0          # shaft x of the front motors; wheelbase is twice this
MOTOR_OUTER_Y = 70.0    # gearbox outer face; the motors overhang the deck by 2 mm
WALL_T, WALL_DROP = 6.0, 26.0
WALL_FROM, WALL_TO = 26.5, 6.0   # wall span, measured back from the shaft
PEG_HOLE_D = 5.0

# Motor pins: a headed pin through each wall hole and gearbox hole, from the
# wheel side. The wall hole is a press fit; the wheel, 2 mm outside the head,
# stops the pin walking out.
PIN_D, PIN_WALL_HOLE = 3.0, 3.0
PIN_HEAD_D, PIN_HEAD_T = 5.5, 1.2
PIN_FLAT = 0.3          # flat along the pin, so it prints lying down

# ---- pillars, locked by wedges -------------------------------------------------
PILLAR_H, PILLAR_D = 38.0, 10.0
PEG_D, PEG_HOLE = 6.0, 6.4      # the pegs on each end, and the holes in the decks
SLOT_W, SLOT_H = 2.8, 4.0       # wedge slot through each peg
SLOT_INTO = 0.4                 # the slot starts this far inside the deck, so the wedge pulls it tight
PEG_CAP = 2.0                   # peg left beyond the slot
PILLAR_FLAT = 2.6               # flat along one side, so the pillar prints lying down
WEDGE_L, WEDGE_T, WEDGE_H0, WEDGE_H1 = 18.0, 2.4, 3.0, 4.4
# (x, y, the direction the wedge slides in). The front pair also locks the
# camera mount's foot, so the camera sits on a direct load path to the bottom
# deck. Nothing stands in x -17..17, the path the battery slides out along.
PILLARS = [(65, 14, (0, 1)), (65, -14, (0, -1)), (30, 36, (0, -1)), (30, -36, (0, 1)),
           (-70, 44, (-1, 0)), (-70, -44, (-1, 0))]

# ---- L298N on snap standoffs ---------------------------------------------------
L298N_CENTRES = [(-42.5, 23), (-42.5, -23)]
L298N_BOARD, L298N_HOLE, L298N_HOLE_D = 43.0, 18.5, 3.0   # 37 mm square hole pattern
SPACER_H, SPACER_D = 5.0, 7.0
DECK_HOLE_SMALL = 3.4

# 3S 2200 mAh size pack, lying crosswise in the gap between front and rear
# wheels, so it slides out sideways for charging without removing anything.
# Two sprung U-clips over it snap through the slots under the deck.
BATTERY = (-17.0, 17.0, 53.0)           # x from, x to, half-width
BATTERY_H = 26.0
STRAP_SLOT_X = (-18.5, 18.5)
STRAP_Y, STRAP_LEN, SLOT_W = 35.0, 22.0, 3.0
WIRE_SLOT = (-30.0, -24.0, 45.5, 50.0)  # x0, x1, y0, y1, mirrored to both sides
BUCK = (35.0, 58.0, 27.0)               # x from, x to, half-length (5 V 5 A module stand-in)
BUCK_H = 20.0
BUCK_TIE_X, BUCK_TIE_Y = (41.0, 52.0), 30.0
CLIP_BOW = {"battery": 1.5, "buck": 0.8}  # the clip's bar bows down this much, and springs over the part

# ---- top deck -----------------------------------------------------------------
Z_BOT_TOP = DECK_T
Z_TOP0 = Z_BOT_TOP + PILLAR_H
Z_TOP_TOP = Z_TOP0 + DECK_T
PI_SD_X = 45.0          # x of the Pi's SD-card edge; USB and Ethernet face the rear
PI_BOSS_D, PI_BOSS_H, PI_HOLE_D = 6.5, 5.0, 2.7
PI_PEG_D, PI_BARB_D = 2.4, 3.1
CABLE_SLOTS = [(0.0, 30.0, 42.0, 50.0), (-10.0, 30.0, -52.0, -44.0)]

# ---- camera mount (ABS) -------------------------------------------------------
BASELINE = 60.0
CAM_HALF_W = 46.0
PLATE_BACK_X, PLATE_T, PLATE_H = 69.0, 4.0, 36.0
FOOT_X0, FOOT_T = 51.0, 5.0
Z_FOOT_TOP = Z_TOP_TOP + FOOT_T
CAM_BOARD_W, CAM_BOARD_H, CAM_BOARD_T = 25.0, 24.0, 1.1
CAM_BOARD_Z0 = Z_FOOT_TOP + 8.0          # room under the board for the ribbon bend
CAM_HOLE_DX, CAM_HOLE_DY, CAM_HOLE_TOP = 21.0, 12.5, 2.0
CAM_BOSS_D, CAM_BOSS_H = 4.5, 3.0        # rear stops behind the board's four holes
GUSSET_T, GUSSET_UP = 4.0, 30.0
TOP_RIB = 6.0
# Rails either side of each board, a ledge under it, a cover in front of it
# (touching only round the four holes), and two tapered keys between the cover
# and the rails' lips.
X_BOARD = PLATE_BACK_X + PLATE_T + CAM_BOSS_H        # back of the camera board
X_BOARD_FRONT = X_BOARD + CAM_BOARD_T
PAD_H, PAD_D, COVER_T = 1.0, 4.0, 1.2    # cover pads clear anything on the board's front
KEY_GAP = 1.3                            # key thickness where it wedges
X_LIP0 = X_BOARD_FRONT + PAD_H + COVER_T + KEY_GAP
X_LIP1 = X_LIP0 + 1.2
RAIL_IN, RAIL_T = CAM_BOARD_W / 2 + 0.15, 1.6
LIP_IN = 10.15                           # lips reach in this far from the camera's centre line
LEDGE_IN = 9.5                           # the 16 mm ribbon passes between the ledges
RAIL_TOP = CAM_BOARD_Z0 + CAM_BOARD_H + 2.0
KEY_W, KEY_L, KEY_T0 = 2.5, 32.0, 0.8
KEY_SLOPE = (KEY_GAP - KEY_T0) / 18.0    # wedges 8 mm above the ledge, so it has room either way
COVER_WINDOW = (12.0, 13.0)              # around the lens

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

    # -- primitives ------------------------------------------------------------
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

    def cone(self, p0, d0, p1, d1):
        return self.tbm.createCylinderOrCone(P(*p0), d0 / 2 * MM, P(*p1), d1 / 2 * MM)

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

    def copy(self, body):
        return self.tbm.copy(body)

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

    def tri_prism_xz(self, cx, cz, lx, lz, y0, y1):
        """Right-angled triangle with the right angle at (cx, cz), legs lx and lz, extruded along y."""
        body = self.box(cx, cx + lx, y0, y1, cz, cz + lz)
        return self.cut_beyond(body, (cx + lx, cz), (cx, cz + lz), y0, y1, (cx, cz))

    # -- printed parts ---------------------------------------------------------
    def bottom_deck(self):
        deck = self.rounded_plate(-DECK_L / 2, DECK_L / 2, -DECK_W / 2, DECK_W / 2, 0, DECK_T, CORNER_R)
        shaft_z = -TT_HEIGHT / 2
        inner_face = MOTOR_OUTER_Y - TT_THICK
        for s in (1, -1):            # front, rear
            for t in (1, -1):        # left, right
                sx = s * MOTOR_X
                wall = self.box(sx - s * WALL_FROM, sx - s * WALL_TO,
                                t * (inner_face - WALL_T), t * inner_face, -WALL_DROP, 1)
                self.union(deck, wall)
                hole_x = sx - s * TT_HOLE_BACK
                for hz in (shaft_z + TT_HOLE_SPACING / 2, shaft_z - TT_HOLE_SPACING / 2):
                    self.cut(deck, self.ycyl(hole_x, hz, t * (inner_face - WALL_T - 1), t * (inner_face + 1),
                                             PIN_WALL_HOLE))
                self.cut(deck, self.ycyl(sx - s * TT_PEG_BACK, shaft_z, t * (inner_face - WALL_T - 1),
                                         t * (inner_face + 1), PEG_HOLE_D))
        for px, py, _ in PILLARS:
            self.cut(deck, self.zcyl(px, py, -1, DECK_T + 1, PEG_HOLE))
        for cx, cy in L298N_CENTRES:
            for dx in (-L298N_HOLE, L298N_HOLE):
                for dy in (-L298N_HOLE, L298N_HOLE):
                    self.cut(deck, self.zcyl(cx + dx, cy + dy, -1, DECK_T + 1, DECK_HOLE_SMALL))
        for x in STRAP_SLOT_X:
            for t in (1, -1):
                self.cut(deck, self.box(x - SLOT_W / 2, x + SLOT_W / 2, t * (STRAP_Y - STRAP_LEN / 2),
                                        t * (STRAP_Y + STRAP_LEN / 2), -1, DECK_T + 1))
        x0, x1, y0, y1 = WIRE_SLOT
        for t in (1, -1):
            self.cut(deck, self.box(x0, x1, t * y0, t * y1, -1, DECK_T + 1))
        # slots for the buck converter's clips
        for x in BUCK_TIE_X:
            for t in (1, -1):
                self.cut(deck, self.box(x - 2, x + 2, t * BUCK_TIE_Y - 1.25, t * BUCK_TIE_Y + 1.25, -1, DECK_T + 1))
        return deck

    def snap_peg(self, x, y, z0, length, d, barb_d, barb_len, slot_w, slot_down, up=True):
        """A split peg with a barb at its tip, for snapping through a board or deck hole.

        The peg starts at z0 and runs ``length`` up (or down), the barb's flat
        catch face at its end; the split runs ``slot_down`` back past z0 into
        whatever the peg stands on, so each half is long enough to flex."""
        s = 1 if up else -1
        end = z0 + s * length
        peg = self.zcyl(x, y, z0, end, d)
        self.union(peg, self.cone((x, y, end), barb_d, (x, y, end + s * barb_len), d - 0.2))
        slot = self.box(x - slot_w / 2, x + slot_w / 2, y - 5, y + 5, z0 - s * slot_down, end + s * (barb_len + 1))
        return peg, slot

    def top_deck(self):
        deck = self.rounded_plate(-DECK_L / 2, DECK_L / 2, -DECK_W / 2, DECK_W / 2, Z_TOP0, Z_TOP_TOP, CORNER_R)
        slots = []
        for hx, hy in pi_holes():
            self.union(deck, self.zcyl(hx, hy, Z_TOP_TOP - 0.5, Z_TOP_TOP + PI_BOSS_H, PI_BOSS_D))
            peg, slot = self.snap_peg(hx, hy, Z_TOP_TOP + PI_BOSS_H, 1.6 + 0.1, PI_PEG_D, PI_BARB_D, 0.9, 0.8, 3.0)
            self.union(deck, peg)
            slots.append(slot)
        self.cut(deck, *slots)
        for px, py, _ in PILLARS:
            self.cut(deck, self.zcyl(px, py, Z_TOP0 - 1, Z_TOP_TOP + 1, PEG_HOLE))
        for x0, x1, y0, y1 in CABLE_SLOTS:
            self.cut(deck, self.box(x0, x1, y0, y1, Z_TOP0 - 1, Z_TOP_TOP + 1))
        return deck

    def camera_mount(self):
        front = PLATE_BACK_X + PLATE_T
        # The foot reaches forward under the rails, so they build up from it.
        mount = self.box(FOOT_X0, X_LIP1, -CAM_HALF_W, CAM_HALF_W, Z_TOP_TOP, Z_FOOT_TOP)
        self.union(mount, self.box(PLATE_BACK_X, front, -CAM_HALF_W, CAM_HALF_W,
                                   Z_TOP_TOP, Z_FOOT_TOP + PLATE_H))
        for y0 in (-GUSSET_T / 2, CAM_HALF_W - GUSSET_T, -CAM_HALF_W):
            self.union(mount, self.tri_prism_xz(PLATE_BACK_X + 0.5, Z_FOOT_TOP - 0.5,
                                                FOOT_X0 - PLATE_BACK_X - 0.5, GUSSET_UP + 0.5,
                                                y0, y0 + GUSSET_T))
        # 45-degree stiffening rib along the top back edge; prints without support
        self.union(mount, self.tri_prism_xz(PLATE_BACK_X + 0.5, Z_FOOT_TOP + PLATE_H, -TOP_RIB - 0.5,
                                            -TOP_RIB - 0.5, -CAM_HALF_W, CAM_HALF_W))
        for cy in (BASELINE / 2, -BASELINE / 2):
            for hy, hz in cam_holes(cy):
                self.union(mount, self.xcyl(hy, hz, front - 0.5, X_BOARD, CAM_BOSS_D))
            top_hole_z = CAM_BOARD_Z0 + CAM_BOARD_H - CAM_HOLE_TOP
            # window for the parts on the back of the camera board ...
            self.cut(mount, self.box(PLATE_BACK_X - 1, front + 1, cy - 7, cy + 7, Z_FOOT_TOP, top_hole_z))
            # ... widened below the lower holes so the 16 mm ribbon can pass through
            self.cut(mount, self.box(PLATE_BACK_X - 1, front + 1, cy - 9.5, cy + 9.5, Z_FOOT_TOP,
                                     top_hole_z - CAM_HOLE_DY - CAM_BOSS_D / 2 - 0.5))
        for cy in (BASELINE / 2, -BASELINE / 2):
            for k in (1, -1):
                # rail wall, lip, and a ledge block under the board's edge; all
                # stand on the foot, so they print without support
                self.union(mount, self.box(front - 0.5, X_LIP1, cy + k * RAIL_IN, cy + k * (RAIL_IN + RAIL_T),
                                           Z_FOOT_TOP - 0.5, RAIL_TOP))
                self.union(mount, self.box(X_LIP0, X_LIP1, cy + k * LIP_IN, cy + k * (RAIL_IN + 0.1),
                                           Z_FOOT_TOP - 0.5, RAIL_TOP))
                self.union(mount, self.box(front - 0.5, X_LIP0, cy + k * LEDGE_IN, cy + k * (RAIL_IN + 0.1),
                                           Z_FOOT_TOP - 0.5, CAM_BOARD_Z0))
        for px, py, _ in PILLARS[:2]:
            self.cut(mount, self.zcyl(px, py, Z_TOP_TOP - 1, Z_FOOT_TOP + 1, PEG_HOLE))
        return mount

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
        """Tapered key between the cover and one rail's lip, wedged in place."""
        x0 = X_BOARD_FRONT + PAD_H + COVER_T
        z0 = RAIL_TOP - (KEY_GAP - 0.01 - KEY_T0) / KEY_SLOPE
        y0, y1 = cy + k * (LIP_IN - 0.15), cy + k * (LIP_IN - 0.15 + KEY_W)
        t1 = KEY_T0 + KEY_L * KEY_SLOPE
        key = self.box(x0, x0 + t1, y0, y1, z0, z0 + KEY_L)
        return self.cut_beyond(key, (x0 + KEY_T0, z0), (x0 + t1, z0 + KEY_L), y0, y1, (x0, z0 + KEY_L / 2))

    def pillar(self, px, py, direction, top_face):
        """A pillar with a peg at each end, and a wedge slot through each peg
        along ``direction``. It has a flat along one side so it prints lying down."""
        ux, uy = direction
        body = self.zcyl(px, py, Z_BOT_TOP, Z_TOP0, PILLAR_D)
        bottom_tip = SLOT_INTO - SLOT_H - PEG_CAP
        top_tip = top_face - SLOT_INTO + SLOT_H + PEG_CAP
        self.union(body, self.zcyl(px, py, bottom_tip, Z_BOT_TOP + 0.5, PEG_D),
                   self.zcyl(px, py, Z_TOP0 - 0.5, top_tip, PEG_D))
        # the slots: bottom one from just inside the bottom deck downwards, top
        # one from just inside the top face upwards
        half_w = SLOT_W / 2
        sx, sy = (5, half_w) if ux else (half_w, 5)
        for z0, z1 in ((SLOT_INTO - SLOT_H, SLOT_INTO), (top_face - SLOT_INTO, top_face - SLOT_INTO + SLOT_H)):
            self.cut(body, self.box(px - sx, px + sx, py - sy, py + sy, z0, z1))
        # flat on the side the wedge's thin end enters from
        if ux:
            x_cut = px - ux * PILLAR_FLAT
            flat = self.box(x_cut, x_cut - ux * 20, py - 20, py + 20, bottom_tip - 1, top_tip + 1)
        else:
            y_cut = py - uy * PILLAR_FLAT
            flat = self.box(px - 20, px + 20, y_cut, y_cut - uy * 20, bottom_tip - 1, top_tip + 1)
        return self.cut(body, flat)

    def wedge(self):
        """Canonical wedge: length along +x from its thin end, thickness along y,
        height up +z from 0 (its bearing face) to the sloped top."""
        w = self.box(0, WEDGE_L, -WEDGE_T / 2, WEDGE_T / 2, 0, WEDGE_H1)
        return self.cut_beyond(w, (0, WEDGE_H0), (WEDGE_L, WEDGE_H1), -WEDGE_T / 2, WEDGE_T / 2, (WEDGE_L / 2, 0))

    def motor_pin(self):
        """Canonical pin along +x: head from -PIN_HEAD_T to 0, shank on to the
        wall's inner face; the flat is underneath (-z)."""
        length = MOTOR_OUTER_Y - (MOTOR_OUTER_Y - TT_THICK - WALL_T)
        pin = self.xcyl(0, 0, -0.01, length, PIN_D)
        self.union(pin, self.xcyl(0, 0, -PIN_HEAD_T, 0, PIN_HEAD_D))
        return self.cut(pin, self.box(-5, length + 5, -5, 5, -10, -PIN_D / 2 + PIN_FLAT))

    def standoff(self):
        """Canonical L298N standoff: stands on the deck top at z = 0."""
        body = self.zcyl(0, 0, 0, SPACER_H, SPACER_D)
        top, top_slot = self.snap_peg(0, 0, SPACER_H, 1.6 + 0.1, 2.7, 3.5, 1.0, 0.9, 2.0)
        bottom, bottom_slot = self.snap_peg(0, 0, 0, DECK_T + 0.1, 3.1, 3.9, 1.2, 1.0, 1.5, up=False)
        self.union(body, top, bottom)
        bottom_slot = self.moved(bottom_slot, rot=(math.pi / 2, (0, 0, 1)))   # split the other way
        return self.cut(body, top_slot, bottom_slot)

    def u_clip(self, kind, along_x, centre, span_half, leg_t, width, top_z):
        """A sprung U-clip: two legs through deck slots, barbs under the deck,
        and a bar that bows down onto the part.

        along_x: the clip spans along x (battery) or y (buck). ``centre`` is its
        position on the other axis; legs are centred at +-span_half."""
        bow = CLIP_BOW[kind]
        bar_t = 2.5
        # Build in a local frame (a, c, z): a along the span, c across it.
        a_out = span_half + leg_t / 2
        c0, c1 = centre - width / 2, centre + width / 2
        clip = None
        # bar, bowed: each half's lower and upper faces slope down to the middle
        for s in (1, -1):
            half = self.box(0, s * a_out, c0, c1, top_z - bow, top_z + bar_t)
            self.cut_beyond(half, (0, top_z - bow), (s * a_out, top_z), c0, c1, (s * a_out / 2, top_z + bar_t))
            self.cut_beyond(half, (0, top_z - bow + bar_t), (s * a_out, top_z + bar_t), c0, c1,
                            (s * a_out / 2, top_z - bow))
            clip = half if clip is None else self.union(clip, half)
        for s in (1, -1):
            a0, a1 = s * (span_half - leg_t / 2), s * (span_half + leg_t / 2)
            self.union(clip, self.box(a0, a1, c0, c1, -3.5, top_z + 0.5))
            # barb: flat catch face just under the deck, lead-in below it
            self.union(clip, self.tri_prism_xz(a1, -0.2, s * 1.3, -3.3, c0, c1))
        if not along_x:
            clip = self.moved(clip, rot=(math.pi / 2, (0, 0, 1)))    # (a, c) -> (y, -x)
        return clip

    # -- stand-ins for bought parts -------------------------------------------
    def motor(self, s, t):
        sx = s * MOTOR_X
        inner = MOTOR_OUTER_Y - TT_THICK
        zc = -TT_HEIGHT / 2
        m = self.box(sx + s * TT_FRONT, sx + s * (TT_FRONT - TT_GEARBOX), t * inner, t * MOTOR_OUTER_Y, -TT_HEIGHT, 0)
        can = self.box(sx + s * (TT_FRONT - TT_GEARBOX), sx + s * (TT_FRONT - TT_LENGTH),
                       t * (inner + 1.9), t * (MOTOR_OUTER_Y - 1.9), zc - 10, zc + 10)
        shaft = self.ycyl(sx, zc, t * (inner - TT_SHAFT_OUT), t * (MOTOR_OUTER_Y + TT_SHAFT_OUT), TT_SHAFT_D)
        peg = self.ycyl(sx - s * TT_PEG_BACK, zc, t * inner, t * (inner - 2), 4.2)
        self.union(m, can, shaft, peg)
        hole_x = sx - s * TT_HOLE_BACK
        for hz in (zc + TT_HOLE_SPACING / 2, zc - TT_HOLE_SPACING / 2):
            self.cut(m, self.ycyl(hole_x, hz, t * (inner - 1), t * (MOTOR_OUTER_Y + 1), TT_HOLE_D))
        return m

    def wheel(self, s, t):
        y0 = MOTOR_OUTER_Y + WHEEL_GAP
        return self.ycyl(s * MOTOR_X, -TT_HEIGHT / 2, t * y0, t * (y0 + WHEEL_W), WHEEL_D)

    def pi5(self):
        z0 = Z_TOP_TOP + PI_BOSS_H
        x0 = PI_SD_X - 85
        pi = self.box(x0, PI_SD_X, -28, 28, z0, z0 + 1.6)
        for hx, hy in pi_holes():
            self.cut(pi, self.zcyl(hx, hy, z0 - 1, z0 + 3, PI_HOLE_D))
        ports = self.box(x0 - 2, x0 + 22, -26, 26, z0 + 1.6, z0 + 17.6)
        cooler = self.box(x0 + 26, PI_SD_X - 8, -22, 22, z0 + 1.6, z0 + 32)
        return self.union(pi, ports, cooler)

    def l298n(self, cx, cy):
        z0 = Z_BOT_TOP + SPACER_H
        board = self.box(cx - L298N_BOARD / 2, cx + L298N_BOARD / 2, cy - L298N_BOARD / 2,
                         cy + L298N_BOARD / 2, z0, z0 + 1.6)
        for dx in (-L298N_HOLE, L298N_HOLE):
            for dy in (-L298N_HOLE, L298N_HOLE):
                self.cut(board, self.zcyl(cx + dx, cy + dy, z0 - 1, z0 + 3, L298N_HOLE_D))
        sink = self.box(cx - 11.5, cx + 11.5, cy - 8, cy + 8, z0 + 1.6, z0 + 27)
        return self.union(board, sink)

    def camera(self, cy):
        board = self.box(X_BOARD, X_BOARD_FRONT, cy - CAM_BOARD_W / 2, cy + CAM_BOARD_W / 2,
                         CAM_BOARD_Z0, CAM_BOARD_Z0 + CAM_BOARD_H)
        lz = lens_z()
        lens = self.box(X_BOARD_FRONT, X_BOARD_FRONT + 5, cy - 4.25, cy + 4.25, lz - 4.25, lz + 4.25)
        return self.union(board, lens)


def pi_holes():
    xs = (PI_SD_X - 3.5, PI_SD_X - 61.5)
    return [(x, y) for x in xs for y in (24.5, -24.5)]


def cam_holes(cy):
    top = CAM_BOARD_Z0 + CAM_BOARD_H - CAM_HOLE_TOP
    return [(cy + dy, z) for dy in (-CAM_HOLE_DX / 2, CAM_HOLE_DX / 2) for z in (top, top - CAM_HOLE_DY)]


def lens_z():
    """Stand-in lens height: midway between the hole rows. Measure the real one."""
    top = CAM_BOARD_Z0 + CAM_BOARD_H - CAM_HOLE_TOP
    return top - CAM_HOLE_DY / 2


def pillar_top_face(i):
    return Z_FOOT_TOP if i < 2 else Z_TOP_TOP


def wedge_offset():
    """How far the wedge's thin end sits from the peg's axis when it is home:
    where it is 3.55 mm tall at the far side of the 6 mm peg (the slot leaves 3.6)."""
    slope = (WEDGE_H1 - WEDGE_H0) / WEDGE_L
    return (SLOT_H - SLOT_INTO - 0.05 - WEDGE_H0) / slope - PEG_D / 2


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

    # -- printed parts, assembled -----------------------------------------------
    structure = {
        "Bottom deck (PLA)": b.bottom_deck(),
        "Top deck (PLA)": b.top_deck(),
        "Camera mount (ABS)": b.camera_mount(),
    }
    pillars = {"Pillar %d" % (i + 1): b.pillar(px, py, d, pillar_top_face(i)) for i, (px, py, d) in enumerate(PILLARS)}

    fasteners = {}
    canonical_wedge = b.wedge()
    off = wedge_offset()
    for i, (px, py, (ux, uy)) in enumerate(PILLARS):
        angle = math.atan2(uy, ux)
        for end, face in (("bottom", 0.0), ("top", pillar_top_face(i))):
            w = b.moved(canonical_wedge, dx=-off)                       # peg axis at the origin
            if end == "bottom":
                w = b.moved(w, rot=(math.pi, (1, 0, 0)))                # bears on the deck's underside
            w = b.moved(w, px, py, face, rot=(angle, (0, 0, 1)))
            fasteners["Wedge %d %s" % (i + 1, end)] = w
    canonical_pin = b.motor_pin()
    inner = MOTOR_OUTER_Y - TT_THICK
    for s, sname in ((1, "front"), (-1, "rear")):
        for t, tname in ((1, "left"), (-1, "right")):
            hole_x = s * MOTOR_X - s * TT_HOLE_BACK
            for k, hz in enumerate((-TT_HEIGHT / 2 + TT_HOLE_SPACING / 2, -TT_HEIGHT / 2 - TT_HOLE_SPACING / 2)):
                # pin axis +x -> -t*y (head outside, pointing inwards)
                pin = b.moved(canonical_pin, rot=(-t * math.pi / 2, (0, 0, 1)))
                fasteners["Motor pin %s %s %d" % (sname, tname, k + 1)] = b.moved(pin, hole_x, t * MOTOR_OUTER_Y, hz)
    canonical_standoff = b.standoff()
    n = 0
    for cx, cy in L298N_CENTRES:
        for dx in (-L298N_HOLE, L298N_HOLE):
            for dy in (-L298N_HOLE, L298N_HOLE):
                n += 1
                fasteners["L298N standoff %d" % n] = b.moved(canonical_standoff, cx + dx, cy + dy, Z_BOT_TOP)
    for t in (1, -1):
        fasteners["Battery clip %s" % ("left" if t > 0 else "right")] = b.u_clip(
            "battery", True, t * STRAP_Y, STRAP_SLOT_X[1], SLOT_W - 0.4, STRAP_LEN - 2, Z_BOT_TOP + BATTERY_H)
    for x in BUCK_TIE_X:
        # spans y; after the 90 deg turn the local "centre" axis c maps to -x
        fasteners["Buck clip x%.0f" % x] = b.u_clip("buck", False, -x, BUCK_TIE_Y, 2.1, 3.6, Z_BOT_TOP + BUCK_H)
    for cy, side in ((BASELINE / 2, "left"), (-BASELINE / 2, "right")):
        fasteners["Camera cover %s" % side] = b.camera_cover(cy)
        for k in (1, -1):
            fasteners["Camera key %s %s" % (side, "outer" if k * cy > 0 else "inner")] = b.camera_key(cy, k)

    ref = {}
    for s, sname in ((1, "front"), (-1, "rear")):
        for t, tname in ((1, "left"), (-1, "right")):
            ref["Motor %s %s" % (sname, tname)] = b.motor(s, t)
            ref["Wheel %s %s" % (sname, tname)] = b.wheel(s, t)
    ref["Pi 5 + Active Cooler"] = b.pi5()
    for cx, cy in L298N_CENTRES:
        ref["L298N %s" % ("left" if cy > 0 else "right")] = b.l298n(cx, cy)
    ref["3S battery (2200 mAh size)"] = b.box(BATTERY[0], BATTERY[1], -BATTERY[2], BATTERY[2], Z_BOT_TOP,
                                              Z_BOT_TOP + BATTERY_H)
    ref["5 V buck converter"] = b.box(BUCK[0], BUCK[1], -BUCK[2], BUCK[2], Z_BOT_TOP, Z_BOT_TOP + BUCK_H)
    ref["Camera left"] = b.camera(BASELINE / 2)
    ref["Camera right"] = b.camera(-BASELINE / 2)

    # -- interference check, before the bodies are handed to Fusion ------------
    def overlap(ba, bb):
        if not ba.boundingBox.intersects(bb.boundingBox):
            return 0.0
        probe = tbm.copy(ba)
        try:
            ok = tbm.booleanOperation(probe, tbm.copy(bb), b.INTERSECT)
            return probe.volume * 1000 if ok else 0.0   # mm^3
        except Exception:
            return 0.0

    # Pairs that touch on purpose: the shaft sits in the wheel hub, and the
    # clips' bars bow down onto the battery and the buck converter to hold them.
    def intended(na, nb):
        if na.startswith("Motor") and not na.startswith("Motor pin") and nb == na.replace("Motor", "Wheel"):
            return True
        for clip, part in (("Battery clip", "3S battery"), ("Buck clip", "5 V buck")):
            if (na.startswith(clip) and nb.startswith(part)) or (nb.startswith(clip) and na.startswith(part)):
                return True
        return False

    everything = list(structure.items()) + list(pillars.items()) + list(fasteners.items()) + list(ref.items())
    clashes = []
    for i in range(len(everything)):
        for j in range(i + 1, len(everything)):
            na, ba = everything[i]
            nb, bb = everything[j]
            if intended(na, nb):
                continue
            vol = overlap(ba, bb)
            if vol > 1.0:
                clashes.append((na, nb, round(vol, 1)))
    # the battery must slide out sideways, between the wheels, without lifting;
    # it pushes the clips' bows up as it goes, so they do not count
    path = b.box(BATTERY[0], BATTERY[1], -150, 150, Z_BOT_TOP + 0.5, Z_BOT_TOP + BATTERY_H)
    blocking = [n for n, body in everything
                if not n.startswith("3S") and not n.startswith("Battery clip") and overlap(path, body) > 1.0]

    # -- assembly ---------------------------------------------------------------
    comps = {}
    for name, body in structure.items():
        occ = new_comp(name)
        occ.component.bRepBodies.add(body).name = name
        comps[name] = occ
    occ = new_comp("Pillars (PLA)")
    for name, body in pillars.items():
        occ.component.bRepBodies.add(body).name = name
    comps["Pillars"] = occ
    occ = new_comp("Printed fasteners (PLA)")
    for name, body in fasteners.items():
        occ.component.bRepBodies.add(body).name = name
    comps["Printed fasteners"] = occ
    refs = new_comp("Bought parts (reference, not printed)").component
    for name, body in ref.items():
        refs.bRepBodies.add(body).name = name
    colour_bodies(app, design, refs)

    stl_dir = os.path.join(out_dir, "stl")
    cad_dir = os.path.join(out_dir, "cad")
    asm_dir = os.path.join(cad_dir, "assembly")
    for d in (stl_dir, cad_dir, asm_dir):
        os.makedirs(d, exist_ok=True)
    for old in os.listdir(stl_dir):
        if old.endswith(".stl"):
            os.remove(os.path.join(stl_dir, old))
    em = design.exportManager

    # printed parts where they sit on the robot, for the Gazebo model
    for fname, key in (("bottom_deck", "Bottom deck (PLA)"), ("top_deck", "Top deck (PLA)"),
                       ("camera_mount", "Camera mount (ABS)"), ("pillars", "Pillars"),
                       ("fasteners", "Printed fasteners")):
        opts = em.createSTLExportOptions(comps[key], os.path.join(asm_dir, fname + ".stl"))
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        em.execute(opts)

    # -- print-oriented copies and exports ------------------------------------
    flip = (math.pi, (1, 0, 0))
    bottom = b.moved(structure["Bottom deck (PLA)"], rot=flip)          # walls up, deck on the bed
    coupon = tbm.copy(structure["Bottom deck (PLA)"])
    tbm.booleanOperation(coupon, b.box(32, 60, 40, DECK_W / 2 + 1, -WALL_DROP - 1, DECK_T + 1), b.INTERSECT)
    coupon = b.moved(coupon, rot=flip)

    def lying_pillar(i):
        """Pillar i, lying on its flat, through-slots vertical."""
        px, py, (ux, uy) = PILLARS[i]
        body = b.moved(pillars["Pillar %d" % (i + 1)], -px, -py, 0)
        if ux:
            body = b.moved(body, rot=(math.pi / 2, (0, 0, 1)))             # slide direction -> y
        return b.moved(body, rot=(math.pi / 2 * (1 if (ux or uy) > 0 else -1), (1, 0, 0)))

    to_print = {
        "motor_fit_test_PLA": coupon,
        "bottom_deck_PLA": bottom,
        "top_deck_PLA": structure["Top deck (PLA)"],
        "camera_mount_ABS": structure["Camera mount (ABS)"],
        "pillar_front_PLA_x2": lying_pillar(0),
        "pillar_PLA_x4": lying_pillar(3),
        "wedge_PLA_x12": b.moved(canonical_wedge, rot=(math.pi / 2, (1, 0, 0))),     # taper in the bed's plane
        "motor_pin_PLA_x8": canonical_pin,
        "l298n_standoff_PLA_x8": canonical_standoff,
        "battery_clip_PLA_x2": b.moved(fasteners["Battery clip left"], rot=(math.pi / 2, (1, 0, 0))),
        "buck_clip_PLA_x2": b.moved(fasteners["Buck clip x41"], rot=(math.pi / 2, (0, 1, 0))),
        "camera_cover_PLA_x2": b.moved(fasteners["Camera cover left"], rot=(math.pi / 2, (0, 1, 0))),
        "camera_key_PLA_x4": b.moved(fasteners["Camera key left outer"], rot=(math.pi / 2, (1, 0, 0))),
    }
    layout_occ = new_comp("Print layout (hidden)")
    layout = layout_occ.component
    exported = []
    next_x = 0.0
    for name, body in to_print.items():
        # onto the bed, laid out in a row so the hidden layout can be inspected
        bb = body.boundingBox
        body = b.moved(body, next_x - bb.minPoint.x / MM, -(bb.minPoint.y + bb.maxPoint.y) / 2 / MM,
                       -bb.minPoint.z / MM)
        next_x += (bb.maxPoint.x - bb.minPoint.x) / MM + 15
        real = layout.bRepBodies.add(body)
        real.name = name
        path = os.path.join(stl_dir, name + ".stl")
        opts = em.createSTLExportOptions(real, path)
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        em.execute(opts)
        bb = real.boundingBox
        size = [round((bb.maxPoint.x - bb.minPoint.x) * 10, 1), round((bb.maxPoint.y - bb.minPoint.y) * 10, 1),
                round((bb.maxPoint.z - bb.minPoint.z) * 10, 1)]
        exported.append((name, size))
    layout_occ.isLightBulbOn = False

    em.execute(em.createFusionArchiveExportOptions(os.path.join(cad_dir, "ster-vis-test-amr.f3d")))
    app.activeViewport.fit()

    return {
        "clashes": clashes,
        "battery_path_blocked_by": blocking,
        "exported": exported,
        "lens_height_above_ground_mm": round(lens_z() + TT_HEIGHT / 2 + WHEEL_D / 2, 1),
        "ground_clearance_mm": round(TT_HEIGHT / 2 + WHEEL_D / 2 - WALL_DROP, 1),
        "lowest_point_above_ground_mm": round(TT_HEIGHT / 2 + WHEEL_D / 2 + (SLOT_INTO - SLOT_H - PEG_CAP), 1),
    }


def colour_bodies(app, design, comp):
    """Best effort: colour the stand-ins so they read apart from printed parts."""
    wanted = {"Motor": "Plastic - Glossy (Yellow)", "Wheel": "Rubber - Soft",
              "Pi": "Paint - Enamel Glossy (Green)", "L298N": "Plastic - Glossy (Red)",
              "3S": "Plastic - Glossy (Blue)", "5 V": "Paint - Enamel Glossy (Green)",
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
