<!--
  Author: Steven (https://github.com/sng516157-web) — 2026-11-03
-->
# Cable insertion sim

Gazebo Harmonic scene of a UR5e arm with a DH AG95 gripper that picks up an SC
plug from a two-post stand and inserts it into an SC port. ROS 2 Jazzy,
`gz_ros2_control`, MoveIt 2, and a deterministic task state machine with
contact-sensor grasp confirmation, a detachable-joint grasp weld, and
weld-time self-calibration for the insertion aim — plus an optional
**camera-based perception pipeline** (classical blob detection or
FoundationPose 6D pose estimation) that replaces the hard-coded plug pose.

> **Status:** the full pipeline runs end to end — pick, level carry, insert,
> release, retreat — and prints `Result: SUCCESS` with tip placement within
> ~2 mm of the port target, with or without perception. Measured perception
> accuracy: ~1 cm (classical, position-only) and sub-millimeter
> (FoundationPose, 6D) against ground truth.

## Supported environment

- Ubuntu 24.04 (WSL 2 with WSLg works; `--cpu` software rendering is the fallback)
- ROS 2 Jazzy (binary packages)
- Gazebo Harmonic via `ros_gz`
- Optional, for `perception:=foundationpose`: an NVIDIA GPU visible to WSL
  (CUDA passthrough) and the conda env from the Perception section

## Setup (Windows + WSL 2)

One-time setup. From Windows PowerShell, a read-only environment check:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\scripts\preflight_windows.ps1
```

Keep two copies of the repo: the **Windows copy** (edit here, git lives here)
and the **Linux runtime copy** (the sim builds and runs here). Never build
under `/mnt/c` directly — it is slow and error-prone.

```bash
# inside Ubuntu (WSL)
mkdir -p ~/cable_ws/src
cp -r /mnt/c/<path-to>/cable-insertion-sim ~/cable_ws/src/
```

Install dependencies (also happens automatically on the first `./scripts/sim.sh`
run):

```bash
sudo apt install \
  ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-ur-description ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
  ros-jazzy-moveit ros-jazzy-gz-msgs-vendor ros-jazzy-gz-transport-vendor \
  ros-jazzy-trac-ik-kinematics-plugin python3-colcon-common-extensions
```

Build:

```bash
cd ~/cable_ws
colcon build --symlink-install --parallel-workers 2
```

`colcon list` should show only `cable_insertion` (`sim/dh_ag95` is a legacy
ROS 1 manifest kept for gripper assets and excluded via `COLCON_IGNORE`).

### After editing files

Edit the **Windows copy**, then sync it over. Either run
`scripts/sync-to-wsl.bat` from Windows (edit the path inside if your checkout
lives elsewhere), or from WSL:

```bash
rsync -a --delete "/mnt/c/<path-to>/cable-insertion-sim/" ~/cable_ws/src/cable-insertion-sim/
cd ~/cable_ws && colcon build --symlink-install --parallel-workers 2
```

With `--symlink-install`, changes to yaml configs, launch files, and Python
scripts need **no rebuild** — just relaunch. Only `src/deterministic_insertion.cpp`
(compiled C++) needs `colcon build`.

## Starting the sim

```bash
./scripts/sim.sh                              # GUI + full deterministic task
./scripts/sim.sh --cpu                        # software rendering (WSLg fallback)
./scripts/sim.sh -- start_task:=false         # simulation only, no task
./scripts/sim.sh -- gui:=false rviz:=false    # headless
./scripts/sim.sh -- rviz:=true                # GUI + RViz with camera feeds
./scripts/sim.sh --cpu -- perception:=classical        # camera-derived plug pose
./scripts/sim.sh --cpu -- perception:=foundationpose   # FoundationPose 6D pose
```

The launch starts Gazebo, spawns the combined UR5e + AG95 robot at
`(-0.15, 0, 0.75)`, and brings up the bridges, controllers, MoveIt, and the
task node. Watch the terminal for the state machine (`==> CLOSE_GRIPPER`,
`Plug welded to gripper`, ... `Result: SUCCESS`).

Choose the arm in `config/arm.env` (`UR_TYPE=ur5e` by default).

## Restarting cleanly (important)

Gazebo and ROS topics are shared machine-wide, and a stale process from a
previous run can reach into a fresh world (this caused real bugs here — e.g. a
stray weld message attaching the plug in a brand-new sim). **Never run two sims
at once**, and clean before each start:

```bash
pkill -9 -f "gz sim"; pkill -9 -f move_group; pkill -9 -f robot_state_publisher
pkill -9 -f gripper_move; pkill -9 -f parameter_bridge; pkill -9 -f pose_bridge
pkill -9 -f static_transform; pkill -9 -f spawner; pkill -9 -f deterministic_insertion
pkill -9 -f plug_perception; pkill -9 -f "ros2 launch"; pkill -9 -f rviz2; sleep 2
# verify nothing is left (should print 0):
ps aux | grep -E "gz sim|ros2 launch|move_group|parameter_bridge" | grep -v grep | wc -l
```

`sim.sh` already runs this cleanup automatically at every launch — the manual
version is for checking what was left over. If the Gazebo window shows a black
viewport or `[WARN:COPY MODE]` in the title, WSLg's GPU state is stale:
`wsl --shutdown` from PowerShell, then relaunch.

If a run dies early with `Cartesian path fraction ... below required` or
`pose plan failed after 3 attempts`, that is the known randomized-planner
flake — the moves retry internally, and a plain relaunch usually passes.
(Planning is slower while the FoundationPose node loads its models on the GPU
at startup; a relaunch clears it.)

## How the task works

```text
START -> OPEN_GRIPPER -> HOME -> PRE_GRASP -> GRASP_APPROACH -> CLOSE_GRIPPER
-> LIFT -> VALIDATE_GRASP -> PRE_INSERT -> PORT_ALIGN -> INSERT
-> VALIDATE_INSERTION -> RELEASE -> RETREAT -> HOME -> SUCCESS
```

- HOME, PRE_GRASP, PRE_INSERT: MoveIt pose planning (OMPL, with bounded
  retries). Grasp approach, lift, port alignment, insertion, retreat:
  Cartesian paths (also with retries).
- The plug rests on two 8 mm posts so the long AG95 fingers can close around
  the housing with the body raised off the table.
- Fingertip + mid-finger **contact sensors** (orange patches are visual-only
  markers) detect the grip. On first confirmed contact the gripper stops
  closing immediately — no over-squeeze — and a gz **detachable joint** welds
  the plug to `wrist_3_link` at exactly the pose it was grasped in. Sim contact
  micro-slip makes friction-only carries of a 40 g plug unreliable; the weld is
  the standard simulation answer. The plugin attaches at spawn by default, so
  the task node detaches once at startup (waiting for the bridge subscription
  first — an immediate publish races DDS discovery) before anything moves, and
  a watchdog undoes any weld that did not come from the task node.
- At the weld instant the task **measures the real plug-tip pose relative to
  the gripper** (ground truth + TF) and re-aims the insertion with it, so
  run-to-run grasp nudges stop mattering.
- The weld is released the moment the insert move finishes; then the gripper
  opens and the arm retreats.
- VALIDATE_GRASP: the plug must rise with the lift. VALIDATE_INSERTION: tip
  within `insertion_pose_tolerance` of the target, or the port contact sensor.

## Perception

The plug pose can come from the cameras instead of the `scene.plug_pose`
config constant. Both estimators publish `/perception/plug_pose`; the task
waits for the first estimate (`perception_timeout`, 10 s classical / 150 s
FoundationPose), uses it, and falls back to the config pose on timeout.
Orientation currently comes from the config either way (the plug's resting
orientation is a known scene constant).

**`perception:=classical`** — `scripts/perception_node.py`. Grey-on-brown blob
segmentation on the plug_cam view, centroid ray projected onto the plug's rest
plane. Position-only, ~1 cm accuracy, zero extra dependencies (numpy + PIL on
the system python).

**`perception:=foundationpose`** — `scripts/perception_fp.py`. RGB + depth from
plug_cam (the depth twin `/plug_cam_depth`), blob mask with a depth gate, then
FoundationPose `register` on the first frame and `track_one` after — full 6D
pose, measured **sub-millimeter** against ground truth in the live sim.

FoundationPose setup (once, on a machine with an NVIDIA GPU):

```bash
# The recipe used here (see the script for the exact steps):
bash ~/perception/install_fp312.sh
# - conda env at python 3.12 (matches ROS Jazzy's rclpy)
# - torch 2.6 cu124, nvdiffrast, pytorch3d built from source
#   (GCC 12 host compiler; nvcc 12.4 chokes on GCC 15 headers)
# - weights: weights/2023-10-28-18-33-37 (refiner) and
#   weights/2024-01-11-20-02-45 (scorer) under ~/perception/FoundationPose/
```

## Cameras

Four cameras: three world cameras (wide workspace, plug stand, port) publish
gz image topics bridged to ROS as `/wide_cam`, `/plug_cam` (+ its depth twin
`/plug_cam_depth`), `/port_cam`, and a wrist-mounted **RGB-D camera** on the
gripper (`/wrist_cam/image`, `/wrist_cam/depth_image`, `/wrist_cam/camera_info`)
on the gripper's open flank with a clear view of the plug in the cage. View
them in RViz (`-- rviz:=true`).

Camera roles: `plug_cam` = detection view (clear before the arm arrives),
`wrist_cam` = grasp-time tracking view, `port_cam` = insertion view. Note the
wrist cam is intentionally not used for pre-grasp detection — the open AG95
fingers block any close mount until the grasp starts.

## Frames

| Frame | Meaning |
|---|---|
| `gazebo_world` | Gazebo world frame; all scene poses in config use it |
| `world` | Robot URDF root / MoveIt planning frame; `gazebo_world` → `world` is the static spawn transform `(-0.15, 0, 0.75)` |
| `gripper_tcp` | Fixed planning tip between the finger pads (`tcp_pose` launch arg) |
| `wrist_cam_link` | Wrist RGB-D camera (fixed to the gripper) |
| plug body | `sc_plug_link`, deterministic pose from config |
| `sc_tip_link` | Plug tip helper frame, fixed offset in the plug model |
| port body | `sc_port_link`, deterministic pose from config |
| `sc_port_base_link_entrance` | Port entrance helper frame, fixed offsets in the port model |

Note: `gazebo_world` only appears in `/tf_static` (as a parent), so TF lookups
*from* it fail — perception publishes in `world` and the task transforms
back by the message's `frame_id`.

## Configuration and tuning

`config/insertion_task.yaml` holds every task number. The ones you will
actually touch:

- `grasp_in_plug: [x, y, z, roll, pitch, yaw]` — where the gripper meets the
  plug, in the plug's frame (plug lies along x: **+x toward the prongs**,
  −x toward the cable; z is height). The launch log prints the resulting world
  pose as `pose GRASP (gazebo): xyz=[0.22+x, -0.12+y, 0.785+z]` so you can
  verify what the sim received. Limits: x no lower than ~−0.008 and z no lower
  than ~0.075, or the pads bottom out on the stand tops before reaching the
  plug (you will see `Pad contact not detected` with the plug untouched).
- `gripper.close_position` — blind close target if contact never fires.
- `scene.stand_a_pose` / `stand_b_pose` — the two support posts (world frame).

`config/controllers.yaml` holds the arm ros2_control controllers. The gripper
keeps its own Gazebo joint controllers and the `/gripper_cmd` topic
(**`0.0` = open, ~`0.63` = closed** — measured on the model; the legacy demo
script's comments have these backwards).

## Simulation simplifications and notes

- `attach_gripper.py` disables gravity on every robot link and adds joint
  damping so the unactuated-by-default setup stays stable; the arm is driven by
  `gz_ros2_control` trajectories.
- The planning scene contains the table, a simplified port collision box, and
  the two stands; the fingers are ACM-allowed against them.
- The plug is carried by the detachable-joint weld, not by friction — see
  "How the task works". The port contact sensor does not currently fire at the
  12 mm insertion depth; validation falls back on tip position.
- The deterministic scene poses are the fallback when perception is off or
  times out; validation always uses ground truth.

## Extension roadmap

1. ~~Camera-derived plug pose~~ — done (classical + FoundationPose). Next:
   port pose from `port_cam`, and orientation from the 6D estimate once the
   plug's near-symmetric yaw is handled.
2. Reactive insertion: contact-guided correction instead of the blind 12 mm
   push (the port and pad contact sensors are already bridged).
3. VLA / learned policy: the deterministic pipeline is the demonstration
   generator, baseline, and reward source; the four camera feeds + action
   primitives are the policy interface.
4. BehaviorTree.CPP or MoveIt Task Constructor for recovery behaviors.

## Layout

```
config/arm.env              which UR arm to spawn
config/controllers.yaml     arm ros2_control controllers
config/insertion_task.yaml  task poses, distances, speeds, tolerances
config/cable_insertion.rviz RViz camera layout
launch/insertion.launch.py  complete startup (sim + MoveIt + task + perception)
moveit_config/              SRDF, kinematics, joint limits, controllers
src/deterministic_insertion.cpp  task state machine
scripts/sim.sh              clean stale processes, install deps, launch
scripts/sync-to-wsl.bat     Windows -> WSL repo sync + rebuild helper
scripts/preflight_*.ps1/sh  read-only environment checks
scripts/pose_bridge.py      gz pose info -> ROS with entity names
scripts/perception_node.py  classical plug detection (plug_cam, position-only)
scripts/perception_fp.py    FoundationPose 6D plug pose (plug_cam RGB-D)
sim/insertion.sdf           table, plug, port, stands, cameras, contact system
sim/models/                 SC plug and SC port meshes
sim/dh_ag95/                DH AG95 meshes, wrist mount, gripper bridge,
                            attach_gripper.py (URDF merge, sensors, weld plugin,
                            wrist camera)
```

## License

Scripts in this repository are MIT ([LICENSE](LICENSE)). The SC meshes are Apache-2.0 ([sim/models/NOTICE](sim/models/NOTICE)). The DH gripper model is BSD ([sim/dh_ag95/NOTICE](sim/dh_ag95/NOTICE)). The Universal Robots description installed by the script keeps its own license. FoundationPose (used by `scripts/perception_fp.py`) is NVIDIA's, under its own license — it is installed separately and not vendored here.
