"""Ster-Vis test AMR: 3D-printed chassis, generated in Fusion 360.

Builds the whole robot in a new direct-modelling design: the printed parts, plus
stand-in models of the bought parts (motors, wheels, Pi 5, L298N drivers,
battery, buck converter, cameras) so fit can be checked. It then runs an
interference check and exports print-oriented STLs and an .f3d archive into
./stl and ./cad next to this file.

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
TT_PEG_BACK = 11.0      # locating peg on the gearbox face, behind the shaft
TT_SHAFT_D, TT_SHAFT_OUT = 5.4, 8.7
WHEEL_D, WHEEL_W, WHEEL_GAP = 65.0, 26.0, 2.0

# ---- bottom deck and motor walls ---------------------------------------------
DECK_L, DECK_W, DECK_T, CORNER_R = 150.0, 136.0, 4.0, 6.0
MOTOR_X = 62.0          # shaft x of the front motors; wheelbase is twice this
MOTOR_OUTER_Y = 70.0    # gearbox outer face; the motors overhang the deck by 2 mm
WALL_T, WALL_DROP = 6.0, 26.0
WALL_FROM, WALL_TO = 26.5, 6.0   # wall span, measured back from the shaft
MOTOR_HOLE_D, MOTOR_SLOT = 3.5, 0.75  # slot lets the holes slide +-0.75 mm
# Captive M3 nut pocket on the inside of each wall. The M3 x 25 bolts go in
# from the wheel side, so only a button head sits between gearbox and wheel.
NUT_AF, NUT_DEPTH = 5.8, 3.0
PEG_HOLE_D = 5.0

PILLAR_H, PILLAR_D = 38.0, 8.0
# The front pair also bolts the camera mount down, so the camera sits on a
# direct load path to the bottom deck. Nothing stands in x -17..17, the path the
# battery slides out along.
PILLARS = [(65, 14), (65, -14), (30, 36), (30, -36), (-70, 30), (-70, -30)]
M3 = 3.4

L298N_CENTRES = [(-44, 23), (-44, -23)]
L298N_BOARD, L298N_HOLE = 43.0, 18.5   # 37 mm square hole pattern
SPACER_H, SPACER_D = 5.0, 7.0

# 3S 2200 mAh size pack, lying crosswise in the gap between front and rear
# wheels, so it slides out sideways for charging without removing anything.
BATTERY = (-17.0, 17.0, 53.0)           # x from, x to, half-width
STRAP_SLOT_X = (-18.5, 18.5)
STRAP_Y, STRAP_LEN, SLOT_W = 35.0, 22.0, 3.0
WIRE_SLOT = (-30.0, -24.0, 45.5, 50.0)  # x0, x1, y0, y1, mirrored to both sides
BUCK = (37.0, 60.0, 27.0)               # x from, x to, half-length (5 V 5 A module stand-in)
BUCK_TIE_X, BUCK_TIE_Y = (43.0, 54.0), 30.0

# ---- top deck -----------------------------------------------------------------
Z_BOT_TOP = DECK_T
Z_TOP0 = Z_BOT_TOP + PILLAR_H
Z_TOP_TOP = Z_TOP0 + DECK_T
PI_SD_X = 45.0          # x of the Pi's SD-card edge; USB and Ethernet face the rear
PI_BOSS_D, PI_BOSS_H, PI_HOLE_D = 6.5, 5.0, 2.8
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
CAM_HOLE_D, CAM_BOSS_D, CAM_BOSS_H = 1.8, 4.5, 3.0   # M2 screws self-tap into ABS
GUSSET_T, GUSSET_UP = 4.0, 30.0
TOP_RIB = 6.0

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

    def rounded_plate(self, x0, x1, y0, y1, z0, z1, r):
        plate = self.box(x0, x1, y0, y1, z0, z1)
        for cx, sx in ((x0, 1), (x1, -1)):
            for cy, sy in ((y0, 1), (y1, -1)):
                self.cut(plate, self.box(cx, cx + sx * r, cy, cy + sy * r, z0 - 1, z1 + 1))
                self.union(plate, self.zcyl(cx + sx * r, cy + sy * r, z0, z1, 2 * r))
        return plate

    def y_slot(self, x, z, y0, y1, d, half_len):
        """Horizontal hole along y, stretched +-half_len along x."""
        s = self.box(x - half_len, x + half_len, y0, y1, z - d / 2, z + d / 2)
        return self.union(s, self.ycyl(x - half_len, z, y0, y1, d), self.ycyl(x + half_len, z, y0, y1, d))

    def hex_y(self, x, z, y0, y1, af):
        """Hexagonal prism along y, vertices up and down so the pocket prints without support."""
        body = None
        for deg in (0, 60, 120):
            a = math.radians(deg)
            obb = adsk.core.OrientedBoundingBox3D.create(
                P(x, (y0 + y1) / 2, z), V(-math.sin(a), 0, math.cos(a)), V(0, 1, 0),
                2 * af * MM, abs(y1 - y0) * MM, af * MM)
            slab = self.tbm.createBox(obb)
            if body is None:
                body = slab
            elif not self.tbm.booleanOperation(body, slab, self.INTERSECT):
                raise RuntimeError("hex failed")
        return body

    def tri_prism_xz(self, cx, cz, lx, lz, y0, y1):
        """Right-angled triangle with the right angle at (cx, cz), legs lx and lz, extruded along y."""
        body = self.box(cx, cx + lx, y0, y1, cz, cz + lz)
        ax, az, bx, bz = cx + lx, cz, cx, cz + lz
        ux, uz = bx - ax, bz - az
        n = math.hypot(ux, uz)
        ux, uz = ux / n, uz / n
        nx, nz = uz, -ux
        mx, mz = (ax + bx) / 2, (az + bz) / 2
        if (mx - cx) * nx + (mz - cz) * nz < 0:
            nx, nz = -nx, -nz
        s = 4 * max(abs(lx), abs(lz))
        obb = adsk.core.OrientedBoundingBox3D.create(
            P(mx + nx * s / 2, (y0 + y1) / 2, mz + nz * s / 2), V(ux, 0, uz), V(nx, 0, nz),
            s * MM, s * MM, (abs(y1 - y0) + 2) * MM)
        return self.cut(body, self.tbm.createBox(obb))

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
                    self.cut(deck, self.y_slot(hole_x, hz, t * (inner_face - WALL_T - 1), t * (inner_face + 1),
                                               MOTOR_HOLE_D, MOTOR_SLOT))
                    # captive nut pocket, open on the inside so the nut slides in
                    self.cut(deck, self.hex_y(hole_x, hz, t * (inner_face - WALL_T + NUT_DEPTH),
                                              t * (inner_face - WALL_T - 6), NUT_AF))
                self.cut(deck, self.ycyl(sx - s * TT_PEG_BACK, shaft_z, t * (inner_face - WALL_T - 1),
                                         t * (inner_face + 1), PEG_HOLE_D))
        for px, py in PILLARS:
            self.cut(deck, self.zcyl(px, py, -1, DECK_T + 1, M3))
        for cx, cy in L298N_CENTRES:
            for dx in (-L298N_HOLE, L298N_HOLE):
                for dy in (-L298N_HOLE, L298N_HOLE):
                    self.cut(deck, self.zcyl(cx + dx, cy + dy, -1, DECK_T + 1, M3))
        for x in STRAP_SLOT_X:
            for t in (1, -1):
                self.cut(deck, self.box(x - SLOT_W / 2, x + SLOT_W / 2, t * (STRAP_Y - STRAP_LEN / 2),
                                        t * (STRAP_Y + STRAP_LEN / 2), -1, DECK_T + 1))
        x0, x1, y0, y1 = WIRE_SLOT
        for t in (1, -1):
            self.cut(deck, self.box(x0, x1, t * y0, t * y1, -1, DECK_T + 1))
        # zip-tie slots either side of the buck converter
        for x in BUCK_TIE_X:
            for t in (1, -1):
                self.cut(deck, self.box(x - 2, x + 2, t * BUCK_TIE_Y - 1.25, t * BUCK_TIE_Y + 1.25, -1, DECK_T + 1))
        return deck

    def top_deck(self):
        deck = self.rounded_plate(-DECK_L / 2, DECK_L / 2, -DECK_W / 2, DECK_W / 2, Z_TOP0, Z_TOP_TOP, CORNER_R)
        for hx, hy in pi_holes():
            self.union(deck, self.zcyl(hx, hy, Z_TOP_TOP - 0.5, Z_TOP_TOP + PI_BOSS_H, PI_BOSS_D))
            self.cut(deck, self.zcyl(hx, hy, Z_TOP0 - 1, Z_TOP_TOP + PI_BOSS_H + 1, PI_HOLE_D))
        for px, py in PILLARS:
            self.cut(deck, self.zcyl(px, py, Z_TOP0 - 1, Z_TOP_TOP + 1, M3))
        for x0, x1, y0, y1 in CABLE_SLOTS:
            self.cut(deck, self.box(x0, x1, y0, y1, Z_TOP0 - 1, Z_TOP_TOP + 1))
        return deck

    def camera_mount(self):
        front = PLATE_BACK_X + PLATE_T
        mount = self.box(FOOT_X0, front, -CAM_HALF_W, CAM_HALF_W, Z_TOP_TOP, Z_FOOT_TOP)
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
                self.union(mount, self.xcyl(hy, hz, front - 0.5, front + CAM_BOSS_H, CAM_BOSS_D))
                self.cut(mount, self.xcyl(hy, hz, PLATE_BACK_X - 0.5, front + CAM_BOSS_H + 1, CAM_HOLE_D))
            top_hole_z = CAM_BOARD_Z0 + CAM_BOARD_H - CAM_HOLE_TOP
            # window for the parts on the back of the camera board ...
            self.cut(mount, self.box(PLATE_BACK_X - 1, front + 1, cy - 7, cy + 7, Z_FOOT_TOP, top_hole_z))
            # ... widened below the lower screws so the 16 mm ribbon can pass through
            self.cut(mount, self.box(PLATE_BACK_X - 1, front + 1, cy - 9.5, cy + 9.5, Z_FOOT_TOP,
                                     top_hole_z - CAM_HOLE_DY - CAM_BOSS_D / 2 - 0.5))
        for px, py in PILLARS[:2]:
            self.cut(mount, self.zcyl(px, py, Z_TOP_TOP - 1, Z_FOOT_TOP + 1, M3))
        return mount

    def tube(self, d, h):
        t = self.zcyl(0, 0, 0, h, d)
        return self.cut(t, self.zcyl(0, 0, -1, h + 1, M3))

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
        return self.union(m, can, shaft, peg)

    def wheel(self, s, t):
        y0 = MOTOR_OUTER_Y + WHEEL_GAP
        return self.ycyl(s * MOTOR_X, -TT_HEIGHT / 2, t * y0, t * (y0 + WHEEL_W), WHEEL_D)

    def pi5(self):
        z0 = Z_TOP_TOP + PI_BOSS_H
        x0 = PI_SD_X - 85
        pi = self.box(x0, PI_SD_X, -28, 28, z0, z0 + 1.6)
        ports = self.box(x0 - 2, x0 + 22, -26, 26, z0 + 1.6, z0 + 17.6)
        cooler = self.box(x0 + 26, PI_SD_X - 8, -22, 22, z0 + 1.6, z0 + 32)
        return self.union(pi, ports, cooler)

    def l298n(self, cx, cy):
        z0 = Z_BOT_TOP + SPACER_H
        board = self.box(cx - L298N_BOARD / 2, cx + L298N_BOARD / 2, cy - L298N_BOARD / 2,
                         cy + L298N_BOARD / 2, z0, z0 + 1.6)
        sink = self.box(cx - 11.5, cx + 11.5, cy - 8, cy + 8, z0 + 1.6, z0 + 27)
        return self.union(board, sink)

    def camera(self, cy):
        x0 = PLATE_BACK_X + PLATE_T + CAM_BOSS_H
        board = self.box(x0, x0 + CAM_BOARD_T, cy - CAM_BOARD_W / 2, cy + CAM_BOARD_W / 2,
                         CAM_BOARD_Z0, CAM_BOARD_Z0 + CAM_BOARD_H)
        lz = lens_z()
        lens = self.box(x0 + CAM_BOARD_T, x0 + CAM_BOARD_T + 5, cy - 4.25, cy + 4.25, lz - 4.25, lz + 4.25)
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

    def translated(body, dx, dy, dz):
        c = tbm.copy(body)
        m = adsk.core.Matrix3D.create()
        m.translation = adsk.core.Vector3D.create(dx * MM, dy * MM, dz * MM)
        tbm.transform(c, m)
        return c

    printed = {
        "Bottom deck (PLA)": b.bottom_deck(),
        "Top deck (PLA)": b.top_deck(),
        "Camera mount (ABS)": b.camera_mount(),
    }
    pillar = b.tube(PILLAR_D, PILLAR_H)
    spacer = b.tube(SPACER_D, SPACER_H)

    placed = dict(printed)
    for i, (px, py) in enumerate(PILLARS):
        placed["Pillar %d" % (i + 1)] = translated(pillar, px, py, Z_BOT_TOP)
    n = 0
    for cx, cy in L298N_CENTRES:
        for dx in (-L298N_HOLE, L298N_HOLE):
            for dy in (-L298N_HOLE, L298N_HOLE):
                n += 1
                placed["Spacer %d" % n] = translated(spacer, cx + dx, cy + dy, Z_BOT_TOP)

    ref = {}
    for s, sname in ((1, "front"), (-1, "rear")):
        for t, tname in ((1, "left"), (-1, "right")):
            ref["Motor %s %s" % (sname, tname)] = b.motor(s, t)
            ref["Wheel %s %s" % (sname, tname)] = b.wheel(s, t)
    ref["Pi 5 + Active Cooler"] = b.pi5()
    for i, (cx, cy) in enumerate(L298N_CENTRES):
        ref["L298N %s" % ("left" if cy > 0 else "right")] = b.l298n(cx, cy)
    ref["3S battery (2200 mAh size)"] = b.box(BATTERY[0], BATTERY[1], -BATTERY[2], BATTERY[2], Z_BOT_TOP, Z_BOT_TOP + 26)
    ref["5 V buck converter"] = b.box(BUCK[0], BUCK[1], -BUCK[2], BUCK[2], Z_BOT_TOP, Z_BOT_TOP + 20)
    ref["Camera left"] = b.camera(BASELINE / 2)
    ref["Camera right"] = b.camera(-BASELINE / 2)

    hw = fasteners(b)

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

    everything = list(placed.items()) + list(ref.items()) + list(hw.items())
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
    # the battery must slide out sideways, between the wheels, without lifting
    path = b.box(BATTERY[0], BATTERY[1], -150, 150, Z_BOT_TOP + 0.5, Z_BOT_TOP + 26)
    blocking = [n for n, body in everything if not n.startswith("3S") and overlap(path, body) > 1.0]

    # -- assembly ---------------------------------------------------------------
    for name, body in printed.items():
        new_comp(name).component.bRepBodies.add(body).name = name
    pillars = new_comp("Pillars (PLA, print 6)").component
    spacers = new_comp("L298N spacers (PLA, print 8)").component
    for name, body in placed.items():
        if name.startswith("Pillar"):
            pillars.bRepBodies.add(body).name = name
        elif name.startswith("Spacer"):
            spacers.bRepBodies.add(body).name = name
    refs = new_comp("Bought parts (reference, not printed)").component
    for name, body in ref.items():
        refs.bRepBodies.add(body).name = name
    colour_bodies(app, design, refs)
    hw_comp = new_comp("Fastener heads and nuts (reference)").component
    for name, body in hw.items():
        hw_comp.bRepBodies.add(body).name = name

    # -- print-oriented copies and exports ------------------------------------
    stl_dir = os.path.join(out_dir, "stl")
    cad_dir = os.path.join(out_dir, "cad")
    os.makedirs(stl_dir, exist_ok=True)
    os.makedirs(cad_dir, exist_ok=True)

    flip = adsk.core.Matrix3D.create()
    flip.setToRotation(math.pi, V(1, 0, 0), P(0, 0, 0))
    bottom = tbm.copy(printed["Bottom deck (PLA)"])
    tbm.transform(bottom, flip)                        # walls up, deck on the bed
    bottom = translated(bottom, 0, 0, DECK_T)
    # one motor wall plus the deck above it: print this first and bolt a motor on
    coupon = tbm.copy(printed["Bottom deck (PLA)"])
    tbm.booleanOperation(coupon, b.box(32, 60, 40, DECK_W / 2 + 1, -WALL_DROP - 1, DECK_T + 1), b.INTERSECT)
    tbm.transform(coupon, flip)
    to_print = {
        "motor_fit_test_PLA": translated(coupon, 0, 0, DECK_T),
        "bottom_deck_PLA": bottom,
        "top_deck_PLA": translated(printed["Top deck (PLA)"], 0, 0, -Z_TOP0),
        "camera_mount_ABS": translated(printed["Camera mount (ABS)"], -(FOOT_X0 + PLATE_BACK_X + PLATE_T) / 2, 0, -Z_TOP_TOP),
        "pillar_PLA_x6": pillar,
        "l298n_spacer_PLA_x8": spacer,
    }
    layout_occ = new_comp("Print layout (hidden)")
    layout = layout_occ.component
    em = design.exportManager
    exported = []
    next_x = 0.0
    for name, body in to_print.items():
        # lay the parts out in a row so the hidden layout can be inspected
        bb = body.boundingBox
        body = translated(body, next_x - bb.minPoint.x / MM, -(bb.minPoint.y + bb.maxPoint.y) / 2 / MM, 0)
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
    }


def fasteners(b):
    """Bolt heads and nuts, so the interference check sees them too."""
    hw = {}
    for i, (px, py) in enumerate(PILLARS):
        top = Z_FOOT_TOP if i < 2 else Z_TOP_TOP
        hw["Pillar bolt head %d" % (i + 1)] = b.zcyl(px, py, top, top + 3, 5.7)
        hw["Pillar nut %d" % (i + 1)] = b.zcyl(px, py, -2.4, 0, 6.4)
    n = 0
    for cx, cy in L298N_CENTRES:
        for dx in (-L298N_HOLE, L298N_HOLE):
            for dy in (-L298N_HOLE, L298N_HOLE):
                n += 1
                z_board = Z_BOT_TOP + SPACER_H + 1.6
                hw["L298N bolt head %d" % n] = b.zcyl(cx + dx, cy + dy, -3, 0, 5.7)
                hw["L298N nut %d" % n] = b.zcyl(cx + dx, cy + dy, z_board, z_board + 2.4, 6.4)
    for i, (hx, hy) in enumerate(pi_holes()):
        hw["Pi bolt head %d" % (i + 1)] = b.zcyl(hx, hy, Z_TOP0 - 2.5, Z_TOP0, 4.5)
    for s in (1, -1):
        for t in (1, -1):
            hole_x = s * MOTOR_X - s * TT_HOLE_BACK
            for k, hz in enumerate((-TT_HEIGHT / 2 + TT_HOLE_SPACING / 2, -TT_HEIGHT / 2 - TT_HOLE_SPACING / 2)):
                hw["Motor bolt head %+d%+d%d" % (s, t, k)] = b.ycyl(hole_x, hz, t * MOTOR_OUTER_Y,
                                                                     t * (MOTOR_OUTER_Y + 1.65), 5.7)
    return hw


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
