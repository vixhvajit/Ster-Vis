"""Write the indoor warehouse world: building, racking, pallets and a stereo drone.

The drone carries the same Ster-Vis head as the rover (two cameras on a 12 cm
bar), flies at shelf height and maps what it sees. Every collision box also
goes to warehouse.json, so the map it builds can be scored against the true
geometry rather than against another estimate.

    python make_warehouse.py        # writes worlds/warehouse.sdf and worlds/warehouse.json
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from make_world import material, static_model

HERE = Path(__file__).parent
WORLD = HERE / "worlds" / "warehouse.sdf"
TRUTH = HERE / "worlds" / "warehouse.json"

# Building: a 20 x 12 m floor with 4 m walls and a flat roof, which is what a
# small distribution unit looks like.
HALF_X, HALF_Y = 10.0, 6.0
HEIGHT = 4.0
WALL_THICKNESS = 0.2
WALL_SEGMENT = 2.0

# Racking: three runs along x, each split by a cross aisle in the middle, with
# 3.3 m aisles between them for the drone.
RACK_Y = (-4.5, 0.0, 4.5)
RACK_DEPTH = 1.2          # y extent of a run
RACK_HEIGHT = 3.2
BAY = 2.0                 # spacing of the uprights
SEGMENT_BAYS = 3          # bays either side of the cross aisle
CROSS_AISLE = 2.0

START = (-8.0, -2.2, 0.12, 0.0)   # x, y, z, yaw: on the floor, it takes off
BASELINE = 0.12
CAM_X, CAM_Z = 0.16, 0.02         # stereo head on the drone body
WIDTH, HEIGHT_PX, HFOV = 640, 480, 1.2
RATE_HZ = 10

truth_boxes: list[dict] = []


def box(name: str, centre, size, yaw: float = 0.0, texture: str = "obstacle_yellow.png",
        solid: bool = True) -> str:
    """A static box, recorded in the truth list so the map can be scored against it."""
    x, y, z = centre
    sx, sy, sz = size
    truth_boxes.append({"name": name, "centre": [round(float(v), 3) for v in (x, y, z)],
                        "half_size": [round(float(v) / 2, 3) for v in (sx, sy, sz)],
                        "yaw": round(float(yaw), 4)})
    geometry = f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"
    if solid:
        return static_model(name, x, y, z, yaw, geometry, texture)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <link name="link">
        <visual name="visual"><geometry>{geometry}</geometry>
        {material(texture)}
        </visual>
      </link>
    </model>"""


def shell() -> list[str]:
    """Floor, four walls and a roof, in segments so the textures keep their scale."""
    parts = []
    for i in range(int(2 * HALF_X / WALL_SEGMENT)):
        along = -HALF_X + WALL_SEGMENT * (i + 0.5)
        for sign in (1, -1):
            parts.append(box(f"wall_y{'p' if sign > 0 else 'm'}_{i}",
                             (along, sign * (HALF_Y + WALL_THICKNESS / 2), HEIGHT / 2),
                             (WALL_SEGMENT, WALL_THICKNESS, HEIGHT), texture="wall.png"))
    for i in range(int(2 * HALF_Y / WALL_SEGMENT)):
        along = -HALF_Y + WALL_SEGMENT * (i + 0.5)
        for sign in (1, -1):
            parts.append(box(f"wall_x{'p' if sign > 0 else 'm'}_{i}",
                             (sign * (HALF_X + WALL_THICKNESS / 2), along, HEIGHT / 2),
                             (WALL_THICKNESS, WALL_SEGMENT, HEIGHT), texture="wall.png"))
    # Floor and roof as thin slabs: a plane has no thickness to score against,
    # and the roof keeps the sky out of the cameras.
    parts.append(box("floor", (0, 0, -0.05), (2 * HALF_X, 2 * HALF_Y, 0.1), texture="ground.png"))
    parts.append(box("roof", (0, 0, HEIGHT + 0.05), (2 * HALF_X, 2 * HALF_Y, 0.1),
                     texture="wall.png", solid=False))
    return parts


def racking(rng: np.random.Generator) -> list[str]:
    """Pallet racking: uprights, beams, and stock on the lower two levels."""
    parts = []
    half_depth = RACK_DEPTH / 2
    for row, y in enumerate(RACK_Y):
        for side in (-1, 1):
            start = side * (CROSS_AISLE / 2)
            for bay in range(SEGMENT_BAYS):
                x0 = start + side * bay * BAY
                x1 = x0 + side * BAY
                name = f"rack{row}_{'p' if side > 0 else 'm'}{bay}"
                for post, x in enumerate((x0, x1)):
                    for post_y in (y - half_depth + 0.06, y + half_depth - 0.06):
                        parts.append(box(f"{name}_post{post}_{post_y > y:d}",
                                         (x, post_y, RACK_HEIGHT / 2), (0.1, 0.1, RACK_HEIGHT),
                                         texture="obstacle_red.png"))
                middle = (x0 + x1) / 2
                for level, z in enumerate((1.3, 2.5)):
                    parts.append(box(f"{name}_beam{level}", (middle, y, z),
                                     (BAY, RACK_DEPTH, 0.09), texture="obstacle_red.png"))
                # Stock: a pallet and a carton per level, sized and placed at
                # random so the rack is not a repeating pattern the matcher
                # could get right for the wrong reason.
                for level, z in enumerate((0.0, 1.35)):
                    if rng.random() < 0.18:
                        continue  # an empty pallet position
                    width = float(rng.uniform(0.7, 1.1))
                    depth = float(rng.uniform(0.7, 1.0))
                    tall = float(rng.uniform(0.6, 0.9))
                    offset = float(rng.uniform(-0.3, 0.3))
                    parts.append(box(f"{name}_pallet{level}", (middle + offset, y, z + 0.06),
                                     (width + 0.1, depth + 0.1, 0.12), texture="obstacle_yellow.png"))
                    parts.append(box(f"{name}_stock{level}", (middle + offset, y, z + 0.12 + tall / 2),
                                     (width, depth, tall),
                                     texture=("obstacle_green.png" if level else "obstacle_yellow.png")))
    return parts


def loose_stock(rng: np.random.Generator, count: int = 7) -> list[str]:
    """Pallets and stacked cartons left in the aisles, for the drone to avoid."""
    parts, placed = [], []
    aisles = (-2.25, 2.25)  # the middle of each aisle between the rack runs
    while len(placed) < count:
        x = float(rng.uniform(-HALF_X + 2.0, HALF_X - 2.0))
        y = float(rng.choice(aisles)) + float(rng.uniform(-0.7, 0.7))
        if math.hypot(x - START[0], y - START[1]) < 2.5:
            continue  # keep the take-off point clear
        if any(math.hypot(x - px, y - py) < 2.5 for px, py in placed):
            continue
        size = (float(rng.uniform(0.7, 1.1)), float(rng.uniform(0.7, 1.1)), float(rng.uniform(0.8, 1.6)))
        yaw = float(rng.uniform(0, math.pi))
        name = f"stack_{len(placed)}"
        parts.append(box(f"{name}_pallet", (x, y, 0.06), (size[0] + 0.15, size[1] + 0.15, 0.12), yaw,
                         texture="obstacle_yellow.png"))
        parts.append(box(name, (x, y, 0.12 + size[2] / 2), size, yaw, texture="obstacle_green.png"))
        placed.append((x, y))
    return parts


def camera(side: str, y: float) -> str:
    return f"""
        <sensor name="{side}_camera" type="camera">
          <pose>{CAM_X} {y:.3f} {CAM_Z} 0 0 0</pose>
          <always_on>1</always_on>
          <update_rate>{RATE_HZ}</update_rate>
          <topic>/stereo/{side}/image</topic>
          <camera name="{side}">
            <horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{WIDTH}</width><height>{HEIGHT_PX}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>30</far></clip>
            <noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>
          </camera>
        </sensor>"""


def visual(name: str, xyz, geometry: str, rgb: str, rpy: str = "0 0 0") -> str:
    x, y, z = xyz
    return f"""
        <visual name="{name}">
          <pose>{x:.4f} {y:.4f} {z:.4f} {rpy}</pose>
          <geometry>{geometry}</geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>
        </visual>"""


def drone() -> str:
    """A quadrotor with the Ster-Vis head, flown by velocity commands.

    The flight controller is not what is being tested, so the model is driven
    through gz's VelocityControl rather than through rotor thrust: commanded
    velocity in, motion out. The rotors are visual.
    """
    x, y, z, yaw = START
    half_b = BASELINE / 2
    arm, rotor = 0.19, "<cylinder><radius>0.105</radius><length>0.012</length></cylinder>"
    body = [
        visual("body", (0, 0, 0), "<box><size>0.26 0.20 0.08</size></box>", "0.15 0.16 0.2"),
        visual("canopy", (0.02, 0, 0.05), "<box><size>0.16 0.14 0.04</size></box>", "0.1 0.4 0.5"),
        # the Ster-Vis head: a bar with the two cameras on it
        visual("stereo_bar", (CAM_X - 0.015, 0, CAM_Z), "<box><size>0.02 0.18 0.022</size></box>",
               "0.15 0.15 0.15"),
        visual("cam_left", (CAM_X - 0.005, half_b, CAM_Z), "<box><size>0.012 0.024 0.024</size></box>",
               "0.05 0.35 0.15"),
        visual("cam_right", (CAM_X - 0.005, -half_b, CAM_Z), "<box><size>0.012 0.024 0.024</size></box>",
               "0.05 0.35 0.15"),
        # the Pi 5 that would run Ster-Vis, on the back of the deck
        visual("pi5", (-0.06, 0, 0.045), "<box><size>0.085 0.056 0.006</size></box>", "0.05 0.45 0.12"),
    ]
    for i, (ax, ay) in enumerate(((arm, arm), (arm, -arm), (-arm, arm), (-arm, -arm))):
        body.append(visual(f"arm_{i}", (ax / 2, ay / 2, 0),
                           "<box><size>0.28 0.022 0.016</size></box>", "0.1 0.1 0.12",
                           rpy=f"0 0 {math.atan2(ay, ax):.4f}"))
        body.append(visual(f"rotor_{i}", (ax, ay, 0.035), rotor, "0.25 0.25 0.3"))
        body.append(visual(f"leg_{i}", (ax * 0.6, ay * 0.6, -0.07),
                           "<box><size>0.02 0.02 0.10</size></box>", "0.1 0.1 0.12"))
    return f"""
    <model name="drone">
      <pose>{x} {y} {z} 0 0 {yaw:.4f}</pose>
      <link name="base_link">
        <inertial>
          <mass>1.4</mass>
          <inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.035</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision"><geometry><box><size>0.42 0.42 0.12</size></box></geometry></collision>
        {''.join(body)}
        {camera("left", half_b)}
        {camera("right", -half_b)}
        <!-- Not on the real drone: a perfect depth camera at the left camera, so
             both the depth and the map it builds can be scored against truth. -->
        <sensor name="truth_depth" type="depth_camera">
          <pose>{CAM_X} {half_b:.3f} {CAM_Z} 0 0 0</pose>
          <always_on>1</always_on>
          <update_rate>{RATE_HZ}</update_rate>
          <topic>/stereo/truth/depth</topic>
          <camera name="truth">
            <horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{WIDTH}</width><height>{HEIGHT_PX}</height><format>R_FLOAT32</format></image>
            <clip><near>0.05</near><far>30</far></clip>
          </camera>
        </sensor>
        <!-- Not on the real drone: a chase camera, so the flight can be watched
             (Gazebo has no GUI on Windows). -->
        <sensor name="chase_camera" type="camera">
          <pose>-1.8 0 0.9 0 0.38 0</pose>
          <always_on>1</always_on>
          <update_rate>{RATE_HZ}</update_rate>
          <topic>/chase/image</topic>
          <camera name="chase">
            <horizontal_fov>1.3</horizontal_fov>
            <image><width>960</width><height>540</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>40</far></clip>
          </camera>
        </sensor>
        <sensor name="bumper" type="contact">
          <always_on>1</always_on><update_rate>50</update_rate>
          <contact><collision>collision</collision></contact>
        </sensor>
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
        <topic>/model/drone/cmd_vel</topic>
      </plugin>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <odom_frame>odom</odom_frame>
        <robot_base_frame>base_link</robot_base_frame>
        <odom_topic>/model/drone/odometry</odom_topic>
        <dimensions>3</dimensions>
      </plugin>
    </model>"""


def lights() -> str:
    """Ceiling lights, so the inside of a closed building is not black."""
    out = []
    for i, x in enumerate((-6.0, -2.0, 2.0, 6.0)):
        out.append(f"""
    <light type="point" name="bay_light_{i}">
      <pose>{x} 0 3.6 0 0 0</pose>
      <diffuse>0.55 0.55 0.52 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>20</range><constant>0.4</constant><linear>0.02</linear><quadratic>0.004</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>""")
    return "".join(out)


def main() -> None:
    rng = np.random.default_rng(11)
    parts = shell() + racking(rng) + loose_stock(rng)
    sdf = f"""<?xml version="1.0" ?>
<!-- GENERATED by make_warehouse.py - edit that, not this.
     Indoor warehouse: racking, stock, and a drone with a Ster-Vis stereo head. -->
<sdf version="1.10">
  <world name="warehouse">
    <physics name="2ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.75 0.75 0.73 1</ambient>
      <background>0.1 0.1 0.12 1</background>
      <shadows>false</shadows>
    </scene>
{lights()}

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
      </link>
    </model>
{''.join(parts)}
{drone()}
  </world>
</sdf>
"""
    WORLD.write_text(sdf, encoding="utf-8")
    TRUTH.write_text(json.dumps({
        "building": {"half_x": HALF_X, "half_y": HALF_Y, "height": HEIGHT},
        "start": START, "baseline_m": BASELINE, "camera_in_body": [CAM_X, BASELINE / 2, CAM_Z],
        "boxes": truth_boxes,
    }, indent=2), encoding="utf-8")
    print(f"wrote {WORLD} ({len(truth_boxes)} boxes)")


if __name__ == "__main__":
    main()
