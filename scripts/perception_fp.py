#!/usr/bin/env python3
# Author: Steven (https://github.com/sng516157-web) — 2026-11-03
"""FoundationPose-backed perception for the SC plug, v2: RGB-D + mask from the
static plug_cam (detection view, clear before the arm arrives) -> 6D plug pose
on /perception/plug_pose (planning 'world' frame).

Interface is identical to perception_node.py (classical v1): same topic. The
task converts frames by the message's frame_id. Swap estimator by launch arg
(perception:=classical|foundationpose).

Why plug_cam and not the wrist camera: the open AG95 blocks any close wrist
mount before the grasp (measured), while plug_cam sees the plug cleanly before
the arm moves — which is exactly when the task wants the pose. The wrist cam
stays for grasp-time tracking work later.

Requires: ~/miniconda3/envs/foundationpose312 (see ~/perception/install_fp*.sh).
"""

import math
import os
import sys

import numpy as np

PLUG_MESH = os.path.expanduser(
    "~/cable_ws/src/cable-insertion-sim/sim/models/sc_plug/sc_plug_visual.glb")

# plug_cam in sim/insertion.sdf (keep in sync). gz camera: forward = link +x.
CAM_POSE = (0.34, -0.30, 1.05, 0.0, 0.89, 2.16)  # x y z roll pitch yaw
# Robot spawn offset (gazebo_world -> world static transform): the planning
# frame is the gazebo frame shifted by this.
SPAWN_OFFSET = np.array([-0.15, 0.0, 0.75])  # world = gz - SPAWN_OFFSET
IMG_W, IMG_H = 640, 480
HFOV = 0.7
FX = IMG_W / (2 * math.tan(HFOV / 2))
VFOV = 2 * math.atan(math.tan(HFOV / 2) * IMG_H / IMG_W)
FY = IMG_H / (2 * math.tan(VFOV / 2))
K_CAM = np.array([[FX, 0, IMG_W / 2], [0, FY, IMG_H / 2], [0, 0, 1.0]])


def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def plug_mask(rgb):
    """Grey plug + posts on the brown table, below the horizon (proven in v1)."""
    r = rgb[:, :, 0].astype(int)
    g = rgb[:, :, 1].astype(int)
    b = rgb[:, :, 2].astype(int)
    h = np.indices(r.shape)[0]
    return (np.abs(r - b) < 25) & (np.abs(g - b) < 20) & (r < 235) & (h > 120)


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image

    try:
        sys.path.insert(0, os.path.expanduser("~/perception/FoundationPose"))
        from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor  # noqa
        import trimesh
    except ImportError as e:
        print(f"FoundationPose env not available: {e}", file=sys.stderr)
        sys.exit(1)

    class FpPerception(Node):
        def __init__(self):
            super().__init__("plug_perception_fp")
            self.pub = self.create_publisher(PoseStamped, "/perception/plug_pose", 10)
            self.rgb = None
            self.depth = None
            self.est = None
            self.cam_R = rotation_matrix(*CAM_POSE[3:])
            self.cam_p = np.array(CAM_POSE[:3])

            self.create_subscription(Image, "/plug_cam", self.on_rgb, 10)
            self.create_subscription(Image, "/plug_cam_depth", self.on_depth, 10)

            mesh = trimesh.load(PLUG_MESH, force="mesh")
            if hasattr(mesh, "to_mesh"):
                mesh = mesh.to_mesh()
            # FP's make_mesh_tensors needs a texture image; the plug GLB is
            # untextured PBR, so attach a plain grey one.
            mat = getattr(getattr(mesh, "visual", None), "material", None)
            if not hasattr(mat, "image") or mat.image is None:
                from PIL import Image as PILImage
                uv = getattr(mesh.visual, "uv", None)
                if uv is None or len(uv) != len(mesh.vertices):
                    uv = np.zeros((len(mesh.vertices), 2))
                mesh.visual = trimesh.visual.TextureVisuals(
                    uv=uv, image=PILImage.new("RGB", (8, 8), (200, 200, 200)))
            self.mesh = mesh
            self.get_logger().info("FoundationPose perception node up (plug_cam)")

        def on_rgb(self, msg):
            self.rgb = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            self.step()

        def on_depth(self, msg):
            self.depth = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(
                msg.height, msg.width)

        def step(self):
            if self.rgb is None or self.depth is None:
                return
            mask = plug_mask(self.rgb)
            # Depth gate: the table region only (camera is 0.3-0.8 m away).
            mask &= (self.depth > 0.02) & (self.depth < 1.0)
            if mask.sum() < 50:
                return
            if self.est is None:
                fp_debug = os.path.expanduser("~/perception/fp_debug")
                os.makedirs(fp_debug, exist_ok=True)
                self.est = FoundationPose(
                    model_pts=self.mesh.vertices, model_normals=self.mesh.vertex_normals,
                    mesh=self.mesh, scorer=ScorePredictor(),
                    refiner=PoseRefinePredictor(), debug_dir=fp_debug)
                pose_cam = self.est.register(
                    K=K_CAM, rgb=self.rgb, depth=self.depth,
                    ob_mask=mask.astype(np.uint8), iteration=5)
            else:
                pose_cam = self.est.track_one(
                    rgb=self.rgb, depth=self.depth, K=K_CAM, iteration=2)
            self.publish(pose_cam)

        def publish(self, pose_cam):
            # FP: OpenCV optical frame (z fwd, x right, y down). gz camera link:
            # x fwd, y left, z up. Columns below are the optical axes in link
            # coords (verified sub-mm against ground truth in the offline test).
            R_conv = np.array([[0.0, 0.0, 1.0],
                               [-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0]])
            pose_link = np.eye(4)
            pose_link[:3, :3] = R_conv @ pose_cam[:3, :3]
            pose_link[:3, 3] = R_conv @ pose_cam[:3, 3]
            # link frame -> gazebo_world (camera's known SDF pose) -> world
            pos_gz = self.cam_p + self.cam_R @ pose_link[:3, 3]
            pos_world = pos_gz - SPAWN_OFFSET
            rot_gz = self.cam_R @ pose_link[:3, :3]

            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(pos_world[0])
            msg.pose.position.y = float(pos_world[1])
            msg.pose.position.z = float(pos_world[2])
            m = rot_gz
            tr = float(np.trace(m))
            if tr > 0:
                s = math.sqrt(tr + 1.0) * 2
                qw, qx, qy, qz = 0.25 * s, (m[2, 1] - m[1, 2]) / s, \
                    (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
            else:
                i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
                j, k = (i + 1) % 3, (i + 2) % 3
                s = math.sqrt(max(1e-9, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
                q = [0.0, 0.0, 0.0, 0.0]
                q[i] = 0.25 * s
                q[j] = (m[j, i] + m[i, j]) / s
                q[k] = (m[k, i] + m[i, k]) / s
                qw = (m[k, j] - m[j, k]) / s
                qx, qy, qz = q[0], q[1], q[2]
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
            self.pub.publish(msg)

    rclpy.init()
    rclpy.spin(FpPerception())


if __name__ == "__main__":
    main()
