# Ster-Vis test AMR (minimal)

A temporary 4WD skid-steer robot for trying Ster-Vis on real hardware, built to
be printed and running in an afternoon. The chassis is **one printed deck**
with the camera bracket in one piece with it, plus two small camera covers and
four keys. Everything else is glued or just sits in place:

| Part | How it is held |
|---|---|
| 4 BO motors | Glued into shallow pockets under the deck, which line them up. Use epoxy, or plenty of hot glue |
| Pi 5, 2 L298Ns | Drop onto posts with pegs through their mounting holes. A dab of hot glue on each peg |
| Battery (Bonka 3S 2200 mAh, 105 × 34 × 25 mm) | Stands on its long edge in a tray and lifts out for charging. The leads leave through a notch at one end. A rubber band or tape over the top keeps it in on bumps |
| 5 V buck converter | Sits in a low rim, with a dab of glue |
| 2 Camera Module 3 | Slide down between rails onto a ledge. A printed cover, pressed by two tapered keys, clamps each board against four stops. It touches the board only around its four holes, and needs no glue |

![Assembly render](render.png)

The plain BO motors have no encoders, so this robot can test obstacle avoidance
but cannot give the poses that mapping needs.

## Print

| File | Qty | Notes |
|---|---|---|
| `stl/deck_PLA.stl` | 1 | 157 × 136 × 38 mm, prints as it sits, with the bracket up. No supports |
| `stl/camera_cover_PLA_x2.stl` | 2 | Front face down, pads up |
| `stl/camera_key_PLA_x4.stl` | 4 | On its side. Print a few spares: they are small |

PLA, 0.2 mm layers, 3 perimeters, 20% infill (100% for the keys). The deck is
about 86 cm³ of solid geometry: roughly 3–4 hours on a typical printer, and
less on a fast one. The motor pockets are shallow recesses in the bottom face,
bridged over, so they need no supports either.

## Before you glue: measure

The bought-part sizes come from published drawings and product listings, not
from your parts. Check with calipers before gluing anything:
- **Camera Module 3:** 25 × 24 mm board, holes 21 mm apart across and 12.5 mm
  apart vertically, top holes 2 mm below the top edge.
- **Battery:** 105 × 34 × 25 mm. The tray leaves 0.5 mm each side.
- **Pi 5 and L298N holes:** the pegs are 2.4 mm (Pi) and 2.6 mm (L298N).

## Build

1. **Motors.** Solder wires and a 0.1 µF capacitor across each motor's
   terminals. Glue each gearbox into its pocket under the deck, flat face up,
   shaft out, motor can pointing to the middle. The pocket sets the position,
   so press it fully in and hold it while the glue sets.
2. **Wiring through the deck.** Take each side's two motor leads up through the
   slot on that side, behind the Pi.
3. **L298Ns.** Press each board onto its four posts, left board on the left.
   Add a dab of hot glue on each peg. The left board drives the left motors
   (front on OUT1/OUT2, rear on OUT3/OUT4), the right board the right motors.
4. **Pi 5.** Press it onto its four posts, SD card towards the left and USB and
   Ethernet facing right. Add a dab of glue on each peg.
5. **Buck converter.** Put it in the rim beside the Pi with a dab of glue.
   Power the Pi from it, not from an L298N's 5 V pin. Connect every ground
   together.
6. **Battery.** Stand it on its long edge in the rear tray, with the leads
   towards the notch. Put a rubber band or tape over it.
7. **Cameras.** Slide each board down between its rails, lens forward, until
   it sits on the ledges. Slide its cover in on top, pads towards the board.
   Push a key down each side, between the cover and the rail's lip, thin end
   first, until it is firm. It wedges within a few millimetres; do not force
   it. Run the ribbons back through the windows to the Pi.
8. **Wheels** on, and **calibrate** with `ster-vis calibrate`. Recalibrate
   whenever a key has been moved.

Also keep the L298N heatsinks off the PLA (they sit 5 mm up on their posts),
fit a fuse and a switch on the battery lead, and cap the motor PWM duty in
software. A full 3S pack would otherwise put about 10.5 V on 6 V motors;
[ster-vis-amr](https://github.com/vixhvajit/ster-vis-amr) caps it at 57%.

## Main dimensions

| | |
|---|---|
| Wheelbase | 124 mm |
| Track, wheel centre to wheel centre | 166 mm |
| Deck underside above the floor | 43 mm |
| Ground clearance | 21 mm, under the motors |
| Lens height | about 70 mm above the floor |
| Left lens | x 81 mm (front), y +30 mm, from the centre of the wheelbase |

## Ster-Vis settings for this robot

- **`--scan-min-height -0.04`:** 3 cm above the floor, with the lenses about 7 cm up.
- **`--scan-max-height 0.05`:** covers everything the robot can hit.
- **`--scan-range-max 2.5`:** matches what a 60 mm baseline can measure.
- **Mount pose:** `--mount 0.08 0.03 0.07 0 0 0`, the left camera on
  `base_link` (on the floor under the centre of the wheelbase). Measure it on
  the built robot.

## Files

| File | What it is |
|---|---|
| `chassis.py` | The design. A Fusion 360 script that builds the assembly, checks it for interferences and writes everything below. Change dimensions here, not in the STLs |
| `stl/*.stl` | Print-ready parts, oriented for the bed |
| `cad/ster-vis-test-amr.f3d` | The Fusion assembly, with stand-ins for the bought parts |
| `cad/assembly/*.stl` | The printed parts where they sit on the robot, for the Gazebo model |
| `render.png` | The picture above |

To regenerate, run `chassis.py` in Fusion (Utilities > Scripts and Add-Ins, or
through the fusion360 MCP bridge). It takes about 20 seconds and returns a
summary. An empty `clashes` list means nothing overlaps, and an empty
`battery_lift_blocked_by` means the battery lifts straight out.

The earlier, bolted two-deck design, and the fully printed-fastener version
after it, are in this folder's git history.
