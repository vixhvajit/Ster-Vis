# Ster-Vis test AMR

A small 4WD skid-steer robot for testing Ster-Vis on real hardware. The chassis is
3D printed: the camera mount in ABS, everything else in PLA. It carries yellow
BO (TT) gear motors on their stock 65 mm wheels, a 3S LiPo, a Raspberry Pi 5,
two L298N drivers and two Camera Module 3 on a 60 mm baseline.

![Assembly render](render.png)

The plain BO motors have no encoders, so this robot can test obstacle avoidance
but cannot give the poses that mapping needs.

## Files

| File | What it is |
|---|---|
| `chassis.py` | The design. A Fusion 360 script that builds the assembly, checks it for interferences and writes everything below. Change dimensions here, not in the STLs. |
| `stl/*.stl` | Print-ready parts, already oriented for the bed |
| `cad/ster-vis-test-amr.f3d` | The Fusion assembly, including stand-in models of the bought parts |
| `render.png` | The picture above |

To regenerate, run `chassis.py` in Fusion (Utilities > Scripts and Add-Ins, or
through the fusion360 MCP bridge). It opens a new design and prints a summary.
An empty `clashes` list means no part overlaps another, including bolt heads and
nuts. An empty `battery_path_blocked_by` means the battery can still slide out.

## Before you print: measure your motors

TT motor clones vary by about 0.5 mm, and the dimensions here come from
published drawings, not from your motors. Measure one motor with calipers and
compare it with the `TT_*` values at the top of `chassis.py`. The two that
matter most:

- `TT_HOLE_SPACING` (17.5): the distance between the two mounting holes, centre
  to centre.
- `TT_HOLE_BACK` (20.2): the distance from the shaft centre to those holes.

The holes in the walls are slotted 0.75 mm each way along the motor, which
absorbs some error in `TT_HOLE_BACK` but none in `TT_HOLE_SPACING`.

Then print `motor_fit_test_PLA.stl` first. It is one motor wall plus the deck
above it, and takes a few minutes. Bolt a motor to it before printing the full
bottom deck.

## Printing

Every part fits a 180 × 180 mm bed. The largest parts are 150 × 136 mm.

| Part | Material | Qty | Notes |
|---|---|---|---|
| `motor_fit_test_PLA` | PLA | 1 | Print first, see above |
| `bottom_deck_PLA` | PLA | 1 | Prints upside down, motor walls up. No supports |
| `top_deck_PLA` | PLA | 1 | Pi bosses up. No supports |
| `pillar_PLA_x6` | PLA | 6 | Print upright |
| `l298n_spacer_PLA_x8` | PLA | 8 | |
| `camera_mount_ABS` | ABS | 1 | Foot on the bed. Use an enclosure and a 5 mm brim. No supports |

Suggested settings: 0.2 mm layers, 4 perimeters, 5 top and bottom layers,
30% infill (40% for the camera mount). The nut pockets and horizontal holes are
shaped to print without supports.

Keep the L298N heatsinks off the PLA: they sit on the spacers, and PLA softens
at about 55–60 °C.

## Hardware

| Qty | Part | Where |
|---|---|---|
| 8 | M3 × 25 button head + M3 nut | Motors. The nuts press into the hex pockets on the inside of each wall; the bolts go in from the wheel side |
| 4 | M3 × 50 + nut | Middle and rear pillars |
| 2 | M3 × 55 (or 60) + nut | Front pillars. These pass through the camera mount's foot as well |
| 8 | M3 × 16 + nut | L298N boards, heads under the bottom deck |
| 4 | M2.5 × 16 + nut | Pi 5, heads under the top deck |
| 8 | M2 × 8 | Cameras. They self-tap into the ABS |
| 2 | 20 mm hook-and-loop strap | Battery |
| 2 | Zip tie, up to 3.5 mm wide | Buck converter |

## Layout

- **Bottom deck:**
  - The battery lies crosswise in the middle, in the gap between the front and rear wheels. Undo the straps and it slides out sideways for charging, with nothing to remove. It is sized for a 3S 2200 mAh pack (about 106 × 34 × 26 mm).
  - Both L298Ns are at the rear. The left board drives the left motors and the right board drives the right motors.
  - The motor wires come up through the slots beside each board.
  - The 5 V buck converter is zip-tied at the front.
- **Top deck:**
  - The Pi 5 sits with its USB and Ethernet ports facing the rear.
  - The GPIO header is on the right. Run wires down through the slot on that side.
  - The camera mount bolts on at the front, through the two front pillars. This gives it a direct path down to the bottom deck.
- **Camera mount:**
  - Both cameras mount on its front face, 60 mm apart.
  - Each ribbon leaves the bottom of its board, passes back through the window behind it, and runs down over the foot to the Pi.

Main dimensions:

| | |
|---|---|
| Wheelbase | 124 mm |
| Track, wheel centre to wheel centre | 170 mm |
| Overall size | about 189 × 196 × 131 mm |
| Ground clearance | 17.8 mm, under the motor walls |
| Lens height | about 118 mm above the floor |

## Ster-Vis settings for this robot

- **Lens height:** about 0.12 m, estimated from the stand-in camera model. Measure the real one after assembly.
- **`--scan-min-height`:** set it to about `-0.09`, which is 3 cm above the floor. The default of -0.25 reaches through the floor and reports it as obstacles.
- **`--scan-range-max 2.5`:** matches what a 60 mm baseline can measure.
- **Mount pose:** `--mount` is the left camera's pose on the parent frame. With `base_link` on the floor under the centre of the chassis, it is roughly `--mount 0.08 0.03 0.12 0 0 0`. Measure this too.

## Assembly order

1. Solder wires and a 0.1 µF capacitor across each motor's terminals.
2. Press the eight M3 nuts into the wall pockets. Bolt on the motors, putting the
   gearbox's locating peg (if it faces the wall) into the round hole.
3. Fit the L298Ns on the spacers, zip-tie the buck converter, and thread the
   battery straps.
4. Stand the pillars on the bottom deck, add the top deck, and put the camera
   mount on the front. Tighten all six pillar bolts.
5. Fit the Pi 5 on its bosses. Screw the cameras to the mount and route the
   ribbons.
6. Push the wheels on last. Stop when each wheel clears the motor bolt heads.
7. Recalibrate if the camera mount is ever knocked or re-tightened.
