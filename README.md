<!--
  Author: Steven (https://github.com/sng516157-web) — 2026-10-30
-->
# Cable insertion sim

Gazebo Harmonic scene of a UR5e arm with a DH AG95 gripper that picks up an SC
plug from a two-post stand and inserts it into an SC port. ROS 2 Jazzy,
`gz_ros2_control`, MoveIt 2, and a deterministic task state machine with
contact-sensor grasp confirmation, a detachable-joint grasp weld, and
weld-time self-calibration for the insertion aim.

> **Status:** the full pipeline runs end to end — pick, level carry, insert,
> release, retreat — and prints `Result: SUCCESS` with tip placement within
> ~2 mm of the port target.

## Supported environment

- Ubuntu 24.04 (WSL 2 with WSLg works; `--cpu` software rendering is the fallback)
- ROS 2 Jazzy (binary packages)
- Gazebo Harmonic via `ros_gz`

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
pkill -9 -f "ros2 launch"; pkill -9 -f rviz2; sleep 2
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
  the task node detaches once at startup before anything moves, and a watchdog
  undoes any weld that did not come from the task node.
- At the weld instant the task **measures the real plug-tip pose relative to
  the gripper** (ground truth + TF) and re-aims the insertion with it, so
  run-to-run grasp nudges stop mattering.
- The weld is released the moment the insert move finishes; then the gripper
  opens and the arm retreats.
- VALIDATE_GRASP: the plug must rise with the lift. VALIDATE_INSERTION: tip
  within `insertion_pose_tolerance` of the target, or the port contact sensor.

## Cameras

Four cameras: three world cameras (wide workspace, plug stand, port) publish
gz image topics bridged to ROS as `/wide_cam`, `/plug_cam` (+ its depth twin
`/plug_cam_depth`), `/port_cam`, and a wrist-mounted **RGB-D camera** on the
gripper (`/wrist_cam/image`, `/wrist_cam/depth_image`, `/wrist_cam/camera_info`)
with a clear view of the plug in the cage. View them in RViz (`-- rviz:=true`).

These feed the perception pipeline: `perception:=classical` runs classical
blob detection on plug_cam (position-only override), `perception:=foundationpose`
runs FoundationPose 6D pose estimation (requires the conda env — see
`~/perception/install_fp312.sh` on a set-up machine). Both publish
`/perception/plug_pose`; the task uses it for the plug pose instead of the
config constant, with the config value as fallback.

## Frames

| Frame | Meaning |
|---|---|
| `gazebo_world` | Gazebo world frame; all scene poses in config use it |
| `world` | Robot URDF root / MoveIt planning frame; `gazebo_world` → `world` is the static spawn transform `(-0.15, 0, 0.75)` |
| `gripper_tcp` | Fixed planning tip between the finger pads (`tcp_pose` launch arg) |
| plug body | `sc_plug_link`, deterministic pose from config |
| `sc_tip_link` | Plug tip helper frame, fixed offset in the plug model |
| port body | `sc_port_link`, deterministic pose from config |
| `sc_port_base_link_entrance` | Port entrance helper frame, fixed offsets in the port model |

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
- Deterministic poses; no perception-driven grasping yet (cameras are in place).

## Extension roadmap

1. Wrist/world cameras → replace `scene.plug_pose` / `scene.port_pose` (keep
   the deterministic poses as fallback). Camera feeds are already bridged.
2. BehaviorTree.CPP or MoveIt Task Constructor for recovery behaviors.
3. Force/torque-guided final insertion.

## Layout

```
config/arm.env              which UR arm to spawn
config/controllers.yaml     arm ros2_control controllers
config/insertion_task.yaml  task poses, distances, speeds, tolerances
config/cable_insertion.rviz RViz camera layout
launch/insertion.launch.py  complete startup (sim + MoveIt + task)
moveit_config/              SRDF, kinematics, joint limits, controllers
src/deterministic_insertion.cpp  task state machine
scripts/sim.sh              clean stale processes, install deps, launch
scripts/sync-to-wsl.bat     Windows -> WSL repo sync + rebuild helper
scripts/preflight_*.ps1/sh  read-only environment checks
sim/insertion.sdf           table, plug, port, stands, cameras, contact system
sim/models/                 SC plug and SC port meshes
sim/dh_ag95/                DH AG95 meshes, wrist mount, gripper bridge,
                            attach_gripper.py (URDF merge, sensors, weld plugin)
```

## License

Scripts in this repository are MIT ([LICENSE](LICENSE)). The SC meshes are Apache-2.0 ([sim/models/NOTICE](sim/models/NOTICE)). The DH gripper model is BSD ([sim/dh_ag95/NOTICE](sim/dh_ag95/NOTICE)). The Universal Robots description installed by the script keeps its own license.
