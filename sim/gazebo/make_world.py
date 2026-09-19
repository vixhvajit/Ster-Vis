"""Write the stereo obstacle-avoidance world: arena, obstacles and the rover.

The rover is a 4-wheel diff-drive with a Ster-Vis head: two cameras on a bar
(the stereo pair) and a Raspberry Pi 5 on the deck.
Obstacle positions also go to obstacles.json so the run can be scored against
ground truth.

    python make_world.py            # writes worlds/stereo_avoid.sdf
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
WORLD = HERE / "worlds" / "stereo_avoid.sdf"
TRUTH = HERE / "worlds" / "obstacles.json"

ARENA = 12.0          # square arena side, metres
WALL_SEGMENT = 2.0    # walls are built from segments so the texture is not stretched
START = (-4.5, -4.5, math.radians(45))

# Stereo head. Baseline 12 cm: the README's pick for a robot looking a few
# metres ahead (+-1.2 cm at 2 m, closest usable ~0.5 m at 128 disparities).
BASELINE = 0.12
CAM_X, CAM_Z = 0.30, 0.17          # in the chassis frame (chassis centre is 0.2 m up)
WIDTH, HEIGHT, HFOV = 640, 480, 1.2  # ~69 deg, close to a Pi Camera Module 3
RATE_HZ = 10


def material(texture: str) -> str:
    return f"""<material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular>
          <pbr><metal><albedo_map>textures/{texture}</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr>
        </material>"""


def static_model(name: str, x: float, y: float, z: float, yaw: float, geometry: str, texture: str) -> str:
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geometry}</geometry></collision>
        <visual name="visual"><geometry>{geometry}</geometry>
        {material(texture)}
        </visual>
      </link>
    </model>"""


def walls() -> list[str]:
    out, half, height = [], ARENA / 2, 1.0
    count = int(ARENA / WALL_SEGMENT)
    for side, (fixed, horizontal) in enumerate(((half, True), (-half, True), (half, False), (-half, False))):
        for i in range(count):
            along = -half + WALL_SEGMENT * (i + 0.5)
            x, y = (along, fixed) if horizontal else (fixed, along)
            yaw = 0.0 if horizontal else math.pi / 2
            geometry = f"<box><size>{WALL_SEGMENT} 0.1 {height}</size></box>"
            out.append(static_model(f"wall_{side}_{i}", x, y, height / 2, yaw, geometry, "wall.png"))
    return out


def floor_tiles(tile: float = 2.0) -> list[str]:
    """Textured floor, in tiles so the texture keeps its scale.

    Visual only (the ground plane does the physics). A flat-grey floor gives
    SGBM nothing to match, and its false matches near the horizon show up in
    the scan as phantom obstacles; a real floor has texture, so this does too.
    """
    out, count = [], int(ARENA / tile) + 2
    for i in range(count):
        for j in range(count):
            x = -tile * count / 2 + tile * (i + 0.5)
            y = -tile * count / 2 + tile * (j + 0.5)
            out.append(f"""
    <model name="floor_{i}_{j}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.001 0 0 0</pose>
      <link name="link">
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>{tile} {tile}</size></plane></geometry>
        {material("ground.png")}
        </visual>
      </link>
    </model>""")
    return out


def obstacles(seed: int = 7, count: int = 16) -> tuple[list[str], list[dict]]:
    rng = np.random.default_rng(seed)
    textures = ("obstacle_red.png", "obstacle_yellow.png", "obstacle_green.png")
    models, truth = [], []
    while len(truth) < count:
        x, y = rng.uniform(-ARENA / 2 + 1.0, ARENA / 2 - 1.0, 2)
        if math.hypot(x - START[0], y - START[1]) < 1.8:
            continue  # keep the start clear
        kind = "box" if rng.random() < 0.6 else "cylinder"
        height = float(rng.uniform(0.5, 1.0))
        if kind == "box":
            sx, sy = rng.uniform(0.3, 0.8, 2)
            radius = math.hypot(sx, sy) / 2
            geometry = f"<box><size>{sx:.3f} {sy:.3f} {height:.3f}</size></box>"
        else:
            r = float(rng.uniform(0.15, 0.35))
            radius = r
            geometry = f"<cylinder><radius>{r:.3f}</radius><length>{height:.3f}</length></cylinder>"
        if any(math.hypot(x - o["x"], y - o["y"]) < radius + o["radius"] + 0.9 for o in truth):
            continue  # leave gaps the rover can fit through
        yaw = float(rng.uniform(0, math.pi))
        name = f"obstacle_{len(truth)}"
        models.append(static_model(name, x, y, height / 2, yaw, geometry, textures[len(truth) % 3]))
        truth.append({"name": name, "kind": kind, "x": round(float(x), 3), "y": round(float(y), 3),
                      "radius": round(radius, 3), "height": round(height, 3),
                      "geometry": geometry, "yaw": round(yaw, 3)})
    return models, truth


def camera(side: str, y: float) -> str:
    return f"""
        <sensor name="{side}_camera" type="camera">
          <pose>{CAM_X} {y:.3f} {CAM_Z} 0 0 0</pose>
          <always_on>1</always_on>
          <update_rate>{RATE_HZ}</update_rate>
          <topic>/stereo/{side}/image</topic>
          <camera name="{side}">
            <horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>30</far></clip>
            <noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>
          </camera>
        </sensor>"""


def box_visual(name: str, xyz: tuple[float, float, float], size: str, rgb: str) -> str:
    x, y, z = xyz
    return f"""
        <visual name="{name}">
          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>
        </visual>"""


def wheel(name: str, x: float, y: float) -> str:
    cyl = "<cylinder><radius>0.1</radius><length>0.05</length></cylinder>"
    return f"""
      <link name="wheel_{name}">
        <pose>{x} {y} -0.1 -1.5707 0 0</pose>
        <inertial><mass>0.5</mass>
          <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.002</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision"><geometry>{cyl}</geometry></collision>
        <visual name="visual"><geometry>{cyl}</geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      </link>
      <joint name="joint_{name}" type="revolute">
        <parent>chassis</parent><child>wheel_{name}</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>"""


def rover() -> str:
    x, y, yaw = START
    half_b = BASELINE / 2
    head = "".join([
        # mast and camera bar
        box_visual("mast", (CAM_X - 0.02, 0, 0.1275), "0.02 0.02 0.055", "0.15 0.15 0.15"),
        box_visual("stereo_bar", (CAM_X - 0.02, 0, CAM_Z), "0.02 0.20 0.025", "0.15 0.15 0.15"),
        box_visual("cam_left_body", (CAM_X - 0.01, half_b, CAM_Z), "0.012 0.025 0.024", "0.05 0.35 0.15"),
        box_visual("cam_right_body", (CAM_X - 0.01, -half_b, CAM_Z), "0.012 0.025 0.024", "0.05 0.35 0.15"),
        # Raspberry Pi 5 (85 x 56 mm board) with its active cooler, on the deck
        box_visual("pi5_board", (-0.12, 0, 0.1015), "0.085 0.056 0.003", "0.05 0.45 0.12"),
        box_visual("pi5_cooler", (-0.125, 0.005, 0.108), "0.045 0.045 0.010", "0.75 0.75 0.78"),
        box_visual("pi5_usb_eth", (-0.087, 0, 0.110), "0.020 0.052 0.016", "0.8 0.8 0.8"),
    ])
    return f"""
    <model name="rover">
      <pose>{x} {y} 0.2 0 0 {yaw:.4f}</pose>
      <link name="chassis">
        <inertial>
          <mass>10.0</mass>
          <inertia><ixx>0.3</ixx><iyy>0.5</iyy><izz>0.6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision"><geometry><box><size>0.6 0.4 0.2</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.6 0.4 0.2</size></box></geometry>
          <material><ambient>0.2 0.3 0.7 1</ambient><diffuse>0.2 0.3 0.7 1</diffuse></material></visual>
        {head}
        {camera("left", half_b)}
        {camera("right", -half_b)}
        <!-- Not on the real robot: a perfect depth camera at the left camera,
             so each stereo frame can be scored against the true depth. -->
        <sensor name="truth_depth" type="depth_camera">
          <pose>{CAM_X} {half_b:.3f} {CAM_Z} 0 0 0</pose>
          <always_on>1</always_on>
          <update_rate>{RATE_HZ}</update_rate>
          <topic>/stereo/truth/depth</topic>
          <camera name="truth">
            <horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R_FLOAT32</format></image>
            <clip><near>0.05</near><far>30</far></clip>
          </camera>
        </sensor>
        <!-- Not on the real robot: a chase camera behind and above the rover,
             so the run can be watched. Gazebo has no GUI on Windows, so this
             is the Gazebo view in the live window and the video. -->
        <sensor name="chase_camera" type="camera">
          <pose>-2.0 0 1.25 0 0.42 0</pose>
          <always_on>1</always_on>
          <update_rate>{RATE_HZ}</update_rate>
          <topic>/chase/image</topic>
          <camera name="chase">
            <horizontal_fov>1.3</horizontal_fov>
            <image><width>960</width><height>540</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>40</far></clip>
          </camera>
        </sensor>
        <!-- bumper: reports any touch so the run can count collisions; gz ignores
             <topic> here and publishes on
             /world/stereo_avoid/model/rover/link/chassis/sensor/bumper/contact -->
        <sensor name="bumper" type="contact">
          <always_on>1</always_on><update_rate>50</update_rate>
          <contact><collision>collision</collision></contact>
        </sensor>
      </link>
      {wheel("fl", 0.2, 0.25)}{wheel("fr", 0.2, -0.25)}{wheel("rl", -0.2, 0.25)}{wheel("rr", -0.2, -0.25)}
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>joint_fl</left_joint><left_joint>joint_rl</left_joint>
        <right_joint>joint_fr</right_joint><right_joint>joint_rr</right_joint>
        <wheel_separation>0.5</wheel_separation>
        <wheel_radius>0.1</wheel_radius>
        <max_linear_acceleration>1.0</max_linear_acceleration>
        <topic>/model/rover/cmd_vel</topic>
        <odom_topic>/model/rover/odometry</odom_topic>
        <tf_topic>/model/rover/tf</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
      </plugin>
    </model>"""


def main() -> None:
    obstacle_models, truth = obstacles()
    sdf = f"""<?xml version="1.0" ?>
<!-- GENERATED by make_world.py - edit that, not this.
     Stereo obstacle avoidance: rover with a Ster-Vis stereo pair + Pi 5. -->
<sdf version="1.10">
  <world name="stereo_avoid">
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
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.9 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.55 0.5 0.45 1</ambient><diffuse>0.55 0.5 0.45 1</diffuse></material></visual>
      </link>
    </model>
{''.join(floor_tiles())}
{''.join(walls())}
{''.join(obstacle_models)}
{rover()}
  </world>
</sdf>
"""
    WORLD.write_text(sdf, encoding="utf-8")
    TRUTH.write_text(json.dumps({"arena": ARENA, "start": START, "baseline_m": BASELINE,
                                 "obstacles": truth}, indent=2), encoding="utf-8")
    print(f"wrote {WORLD} ({len(truth)} obstacles)")


if __name__ == "__main__":
    main()
