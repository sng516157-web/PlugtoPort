"""Complete simulation + application startup for the deterministic insertion task.

Author: Steven (https://github.com/sng516157-web) — 2026-10-30

Starts, in order:
  1. Gazebo with sim/insertion.sdf
  2. The combined UR5e + AG95 robot description (xacro + attach_gripper.py)
  3. Robot spawning at (-0.15, 0, 0.75), with retry until the world is ready
  4. robot_state_publisher and the gazebo_world -> world static transform
  5. ros_gz_bridge (/clock, pose info, port contact sensor)
  6. Controller spawners (joint_state_broadcaster, joint_trajectory_controller)
  7. The gripper bridge node (gripper_move.py)
  8. MoveIt move_group
  9. Optional RViz
 10. The deterministic task node (start_task:=true by default)
"""

import os
import subprocess
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SPAWN = {"x": "-0.15", "y": "0", "z": "0.75"}


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def generate_launch_description():
    pkg = get_package_share_directory("cable_insertion")
    models = os.path.join(pkg, "sim", "models")

    args = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("start_task", default_value="true"),
        DeclareLaunchArgument(
            "tcp_pose",
            default_value="0 0 0.12 0 0 0",
            description="gripper_tcp pose relative to gripper_base_link",
        ),
    ]

    resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        models + os.pathsep + "/opt/ros/jazzy/share"
        + os.pathsep + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    return LaunchDescription(args + [resource_path, OpaqueFunction(function=setup)])


def setup(context, *_args, **_kwargs):
    pkg = get_package_share_directory("cable_insertion")
    world = os.path.join(pkg, "sim", "insertion.sdf")
    task_yaml = os.path.join(pkg, "config", "insertion_task.yaml")
    controllers_yaml = os.path.join(pkg, "config", "controllers.yaml")
    moveit_dir = os.path.join(pkg, "moveit_config")

    gui = LaunchConfiguration("gui").perform(context) == "true"
    rviz = LaunchConfiguration("rviz").perform(context) == "true"
    start_task = LaunchConfiguration("start_task").perform(context) == "true"
    tcp_pose = LaunchConfiguration("tcp_pose").perform(context)

    # --- 2. Generate the combined robot description -------------------------
    ur_type = os.environ.get("UR_TYPE", "ur5e")
    xacro_file = os.path.join(
        get_package_share_directory("ur_description"), "urdf", "ur.urdf.xacro"
    )
    fd, urdf_path = tempfile.mkstemp(suffix=".urdf")
    os.close(fd)
    with open(urdf_path, "w", encoding="utf-8") as out:
        subprocess.run(
            ["xacro", xacro_file, f"ur_type:={ur_type}", "name:=ur"],
            stdout=out,
            check=True,
        )
    subprocess.run(
        [
            "python3",
            os.path.join(pkg, "sim", "dh_ag95", "attach_gripper.py"),
            urdf_path,
            os.path.join(pkg, "sim", "dh_ag95", "dh_robotics_ag95.urdf"),
            os.path.join(pkg, "sim", "dh_ag95", "meshes"),
            "--ros2-control",
            controllers_yaml,
            "--tcp",
            tcp_pose,
        ],
        check=True,
    )
    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    with open(
        os.path.join(moveit_dir, "cable_insertion.srdf"), "r", encoding="utf-8"
    ) as handle:
        robot_description_semantic = handle.read()

    common = {"use_sim_time": True}

    actions = []

    # --- 1. Gazebo ----------------------------------------------------------
    gz_args = f"-r {world}" if gui else f"-r -s {world}"
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
        )
    )

    # --- 3. Spawn the robot once the world accepts the request --------------
    spawn = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "for i in $(seq 1 30); do "
            "ros2 run ros_gz_sim create -world insertion -name ur "
            f"-file {urdf_path} -x {SPAWN['x']} -y {SPAWN['y']} -z {SPAWN['z']} "
            "&& exit 0; sleep 2; done; "
            "echo 'Robot spawn failed after 30 attempts' >&2; exit 1",
        ],
        output="screen",
    )
    actions.append(spawn)

    # --- 4. TF: gazebo_world -> robot root 'world' ---------------------------
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="gazebo_world_tf",
            arguments=[
                "--x", SPAWN["x"], "--y", SPAWN["y"], "--z", SPAWN["z"],
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "gazebo_world",
                "--child-frame-id", "world",
            ],
            parameters=[common],
        )
    )

    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}, common],
        )
    )

    # --- 5. Bridges ----------------------------------------------------------
    task_params = load_yaml(task_yaml)["deterministic_insertion"]["ros__parameters"]
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                f"{task_params['topics']['port_contact']}"
                "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                "/wide_cam@sensor_msgs/msg/Image[gz.msgs.Image",
                "/plug_cam@sensor_msgs/msg/Image[gz.msgs.Image",
                "/port_cam@sensor_msgs/msg/Image[gz.msgs.Image",
                "/finger1_pad_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                "/finger2_pad_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                "/finger1_mid_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                "/finger2_mid_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                "/plug_weld/attach@std_msgs/msg/Empty]gz.msgs.Empty",
                "/plug_weld/detach@std_msgs/msg/Empty]gz.msgs.Empty",
                "/plug_weld/state@std_msgs/msg/String[gz.msgs.String",
                "/wrist_cam/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/wrist_cam/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/wrist_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            parameters=[common],
            output="screen",
        )
    )
    # Pose ground truth with entity names preserved (ros_gz_bridge drops them).
    actions.append(
        Node(
            package="cable_insertion",
            executable="pose_bridge.py",
            name="gz_pose_bridge",
            parameters=[
                {"ros_topic": task_params["topics"]["pose_info"]},
                common,
            ],
            output="screen",
        )
    )

    # --- 6..10. Everything that needs the spawned robot ----------------------
    # Ordered chain with readiness gates: spawn -> settle -> joint_state_broadcaster
    # -> joint_trajectory_controller -> move_group + task node. Each spawner exits
    # once its controller is active; a non-zero exit stops the chain.
    def after_spawn(event, _context):
        if event.returncode != 0:
            print("[insertion.launch] Robot spawn failed; not starting controllers, "
                  "MoveIt, or the task node.")
            return []
        gripper = Node(
            package="cable_insertion",
            executable="gripper_move.py",
            name="dh_gripper",
            parameters=[common],
            output="screen",
        )
        jsb_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager-timeout", "120",
            ],
            parameters=[common],
            output="screen",
        )

        def after_jsb(event, _context):
            if event.returncode != 0:
                print("[insertion.launch] joint_state_broadcaster failed; stopping.")
                return []
            jtc_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_trajectory_controller",
                    "--controller-manager-timeout", "120",
                ],
                parameters=[common],
                output="screen",
            )
            return [jtc_spawner, RegisterEventHandler(
                OnProcessExit(target_action=jtc_spawner, on_exit=after_jtc)
            )]

        def after_jtc(event, _context):
            if event.returncode != 0:
                print("[insertion.launch] joint_trajectory_controller failed; "
                      "not starting MoveIt or the task node.")
                return []
            return move_group_node() + task_node()

        return [
            gripper,
            TimerAction(period=4.0, actions=[jsb_spawner]),
            RegisterEventHandler(
                OnProcessExit(target_action=jsb_spawner, on_exit=after_jsb)
            ),
        ]

    def move_group_node():
        moveit_params = {
            "robot_description": robot_description,
            "robot_description_semantic": robot_description_semantic,
            "robot_description_kinematics": load_yaml(
                os.path.join(moveit_dir, "kinematics.yaml")
            ),
            "robot_description_planning": load_yaml(
                os.path.join(moveit_dir, "joint_limits.yaml")
            ),
            "planning_pipelines": ["ompl"],
            "ompl": load_yaml(os.path.join(moveit_dir, "ompl_planning.yaml")),
            "planning_scene_monitor": {
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
            "trajectory_execution": {
                "moveit_manage_controllers": True,
                "allowed_execution_duration_scaling": 2.0,
                "allowed_goal_duration_margin": 1.0,
                "allowed_start_tolerance": 0.05,
            },
            **load_yaml(os.path.join(moveit_dir, "moveit_controllers.yaml")),
            **common,
        }
        nodes = [
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[moveit_params],
                output="screen",
            )
        ]
        if rviz:
            nodes.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=[
                        "-d",
                        os.path.join(
                            get_package_share_directory("cable_insertion"),
                            "config",
                            "cable_insertion.rviz",
                        ),
                    ],
                    parameters=[
                        {
                            "robot_description": robot_description,
                            "robot_description_semantic": robot_description_semantic,
                        },
                        common,
                    ],
                )
            )
        return nodes

    def task_node():
        if not start_task:
            return []
        return [
            Node(
                package="cable_insertion",
                executable="deterministic_insertion",
                parameters=[
                    task_yaml,
                    {
                        "robot_description": robot_description,
                        "robot_description_semantic": robot_description_semantic,
                        "robot_description_planning": load_yaml(
                            os.path.join(moveit_dir, "joint_limits.yaml")
                        ),
                    },
                    common,
                ],
                output="screen",
            )
        ]

    actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)))

    return actions
