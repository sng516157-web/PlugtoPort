#!/usr/bin/env python3
# Author: Steven (https://github.com/sng516157-web) — 2026-11-03
"""FoundationPose perception for the SC plug: RGB-D + mask from plug_cam
(detection view) and the wrist camera (tracking view) -> continuous 6D plug
pose on /perception/plug_pose and an expiring RViz marker.

Tracking handoff: plug_cam registers/ tracks while the plug is on the stand;
once the plug enters the wrist camera's forward view, tracking hands off
(est.pose_last is re-expressed in the wrist optical frame) so the estimate
follows the plug through the grasp and carry. If the handoff can't convert,
a fresh register is used. Estimates that jump > 5 cm from the last accepted
pose are rejected.

Requires: ~/miniconda3/envs/foundationpose312 (see ~/perception/install_fp312.sh).
"""

import math
import os
import sys

import numpy as np

PLUG_MESH = os.path.expanduser(
    "~/cable_ws/src/cable-insertion-sim/sim/models/sc_plug/sc_plug_visual.glb")

# plug_cam in sim/insertion.sdf (keep in sync). gz camera: forward = link +x.
CAM_POSE = (0.34, -0.30, 1.05, 0.0, 0.89, 2.16)  # x y z roll pitch yaw
# Robot spawn offset (gazebo_world -> world static transform).
SPAWN_OFFSET = np.array([-0.15, 0.0, 0.75])  # world = gz - SPAWN_OFFSET
IMG_W, IMG_H = 640, 480
HFOV = 0.7
FX = IMG_W / (2 * math.tan(HFOV / 2))
VFOV = 2 * math.atan(math.tan(HFOV / 2) * IMG_H / IMG_W)
FY = IMG_H / (2 * math.tan(VFOV / 2))
K_PLUG = np.array([[FX, 0, IMG_W / 2], [0, FY, IMG_H / 2], [0, 0, 1.0]])
HFOV_WRIST = 1.0
FXW = IMG_W / (2 * math.tan(HFOV_WRIST / 2))
VFOVW = 2 * math.atan(math.tan(HFOV_WRIST / 2) * IMG_H / IMG_W)
FYW = IMG_H / (2 * math.tan(VFOVW / 2))
K_WRIST_FALLBACK = np.array([[FXW, 0, IMG_W / 2], [0, FYW, IMG_H / 2], [0, 0, 1.0]])

# optical (FP output: z fwd, x right, y down) -> gz camera link (x fwd, y left,
# z up): columns are the optical axes in link coords (verified vs ground truth).
R_CONV = np.array([[0.0, 0.0, 1.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0]])


def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def mask_plugcam(rgb):
    """Grey plug + posts on the brown table, below the horizon."""
    r = rgb[:, :, 0].astype(int)
    g = rgb[:, :, 1].astype(int)
    b = rgb[:, :, 2].astype(int)
    h = np.indices(r.shape)[0]
    return (np.abs(r - b) < 25) & (np.abs(g - b) < 20) & (r < 235) & (h > 120)


def mask_wrist(rgb):
    """Light-grey plug between dark fingers (nose view); brightness band so the
    white wall and light arm links are excluded."""
    r = rgb[:, :, 0].astype(int)
    g = rgb[:, :, 1].astype(int)
    b = rgb[:, :, 2].astype(int)
    bright = (r + g + b) / 3
    return (bright > 110) & (bright < 235) & (np.abs(r - b) < 30) & (np.abs(g - b) < 25)


def to_np(m):
    """FP returns torch tensors in places — normalize to float64 numpy."""
    if hasattr(m, "detach"):
        m = m.detach().cpu().numpy()
    return np.asarray(m, dtype=np.float64)


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image, CameraInfo
    from visualization_msgs.msg import Marker
    import tf2_ros

    try:
        sys.path.insert(0, os.path.expanduser("~/perception/FoundationPose"))
        from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor  # noqa
        import trimesh
        import torch
    except ImportError as e:
        print(f"FoundationPose env not available: {e}", file=sys.stderr)
        sys.exit(1)

    T_OPT_TO_LINK = np.eye(4)
    T_OPT_TO_LINK[:3, :3] = R_CONV
    T_LINK_TO_OPT = np.linalg.inv(T_OPT_TO_LINK)

    class FpPerception(Node):
        def __init__(self):
            super().__init__("plug_perception_fp")
            self.pub = self.create_publisher(PoseStamped, "/perception/plug_pose", 10)
            self.marker_pub = self.create_publisher(
                Marker, "/perception/plug_marker", 10)
            self.tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(self.tf_buffer, self)

            self.frames = {  # src -> {"rgb":, "depth":, "K":}
                "plug": {"rgb": None, "depth": None, "K": K_PLUG},
                "wrist": {"rgb": None, "depth": None, "K": K_WRIST_FALLBACK},
            }
            self.est = None
            self.src = None
            self.last_pos_world = None

            self.create_subscription(Image, "/plug_cam", self.mk_rgb("plug"), 10)
            self.create_subscription(
                Image, "/plug_cam_depth", self.mk_depth("plug"), 10)
            self.create_subscription(Image, "/wrist_cam/image", self.mk_rgb("wrist"), 10)
            self.create_subscription(
                Image, "/wrist_cam/depth_image", self.mk_depth("wrist"), 10)
            self.create_subscription(
                CameraInfo, "/wrist_cam/camera_info", self.on_wrist_info, 10)

            mesh = trimesh.load(PLUG_MESH, force="mesh")
            if hasattr(mesh, "to_mesh"):
                mesh = mesh.to_mesh()
            mat = getattr(getattr(mesh, "visual", None), "material", None)
            if not hasattr(mat, "image") or mat.image is None:
                from PIL import Image as PILImage
                uv = getattr(mesh.visual, "uv", None)
                if uv is None or len(uv) != len(mesh.vertices):
                    uv = np.zeros((len(mesh.vertices), 2))
                mesh.visual = trimesh.visual.TextureVisuals(
                    uv=uv, image=PILImage.new("RGB", (8, 8), (200, 200, 200)))
            self.mesh = mesh

            self.cam_R = rotation_matrix(*CAM_POSE[3:])
            self.cam_T = np.eye(4)
            self.cam_T[:3, :3] = self.cam_R
            self.cam_T[:3, 3] = CAM_POSE[:3]
            self.get_logger().info("FoundationPose perception node up (plug_cam + wrist)")

        def mk_rgb(self, src):
            def cb(msg):
                self.frames[src]["rgb"] = np.frombuffer(
                    bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
                self.step()
            return cb

        def mk_depth(self, src):
            def cb(msg):
                self.frames[src]["depth"] = np.frombuffer(
                    bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
                self.step()
            return cb

        def on_wrist_info(self, msg):
            self.frames["wrist"]["K"] = np.array(msg.k).reshape(3, 3)

        def world_from_camopt(self, pose_cam_opt, src):
            """optical frame -> world, per camera source."""
            pose_link = T_OPT_TO_LINK @ to_np(pose_cam_opt)
            if src == "plug":
                return self.cam_T @ pose_link
            try:
                tf = self.tf_buffer.lookup_transform(
                    "world", "wrist_cam_link", rclpy.time.Time())
            except Exception:
                return None
            T = np.eye(4)
            T[:3, :3] = self._quat_mat(tf.transform.rotation)
            T[:3, 3] = [tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z]
            return T @ pose_link

        @staticmethod
        def _quat_mat(q):
            x, y, z, w = q.x, q.y, q.z, q.w
            return np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])

        def step(self):
            # Prefer the wrist view when it sees the plug (it follows the
            # gripper); fall back to the static plug_cam.
            src = None
            wrist = self.frames["wrist"]
            if wrist["rgb"] is not None and wrist["depth"] is not None:
                if (mask_wrist(wrist["rgb"]) & (wrist["depth"] > 0.02)
                        & (wrist["depth"] < 0.30)).sum() > 300:
                    src = "wrist"
            if src is None:
                plug = self.frames["plug"]
                if plug["rgb"] is not None and plug["depth"] is not None:
                    if (mask_plugcam(plug["rgb"]) & (plug["depth"] > 0.02)
                            & (plug["depth"] < 1.0)).sum() > 300:
                        src = "plug"
            if src is None:
                return
            f = self.frames[src]

            if self.est is None:
                fp_debug = os.path.expanduser("~/perception/fp_debug")
                os.makedirs(fp_debug, exist_ok=True)
                self.est = FoundationPose(
                    model_pts=self.mesh.vertices,
                    model_normals=self.mesh.vertex_normals, mesh=self.mesh,
                    scorer=ScorePredictor(), refiner=PoseRefinePredictor(),
                    debug_dir=fp_debug)
                mask = (mask_wrist(f["rgb"]) if src == "wrist"
                        else mask_plugcam(f["rgb"]))
                mask &= (f["depth"] > 0.02)
                mask &= (f["depth"] < (0.30 if src == "wrist" else 1.0))
                pose_cam = self.est.register(
                    K=f["K"], rgb=f["rgb"], depth=f["depth"],
                    ob_mask=mask.astype(np.uint8), iteration=5)
            else:
                if src != self.src:
                    # Camera switch: re-register on the new view rather than
                    # re-expressing pose_last across frames (frame-chain errors
                    # there produced 0.29 m jumps). Register gives a fresh
                    # global pose; tracking then continues at full rate.
                    self._register(src, f)
                    return
                mask = (mask_wrist(f["rgb"]) if src == "wrist"
                        else mask_plugcam(f["rgb"]))
                mask &= (f["depth"] > 0.02)
                mask &= (f["depth"] < (0.30 if src == "wrist" else 1.0))
                pose_cam = self.est.track_one(
                    rgb=f["rgb"], depth=f["depth"], K=f["K"], iteration=2)

            pose_world = self.world_from_camopt(pose_cam, src)
            if pose_world is None:
                return
            self.src = src
            if self.last_pos_world is not None:
                if np.linalg.norm(pose_world[:3, 3] - self.last_pos_world) > 0.05:
                    return  # bad frame, keep last
            self.last_pos_world = pose_world[:3, 3].copy()
            self.publish(pose_world)

        def _register(self, src, f):
            mask = (mask_wrist(f["rgb"]) if src == "wrist"
                    else mask_plugcam(f["rgb"]))
            mask &= (f["depth"] > 0.02)
            mask &= (f["depth"] < (0.30 if src == "wrist" else 1.0))
            pose_cam = self.est.register(
                K=f["K"], rgb=f["rgb"], depth=f["depth"],
                ob_mask=mask.astype(np.uint8), iteration=5)
            pose_world = self.world_from_camopt(pose_cam, src)
            if pose_world is None:
                return
            self.src = src
            self.last_pos_world = pose_world[:3, 3].copy()
            self.publish(pose_world)

        def publish(self, pose_world):
            pos_world = pose_world[:3, 3] - SPAWN_OFFSET
            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(pos_world[0])
            msg.pose.position.y = float(pos_world[1])
            msg.pose.position.z = float(pos_world[2])
            m = pose_world[:3, :3]
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

            mk = Marker()
            mk.header.frame_id = "world"
            mk.header.stamp = msg.header.stamp
            mk.ns = "plug"
            mk.id = 0
            mk.type = Marker.ARROW
            mk.action = Marker.ADD
            mk.lifetime = rclpy.duration.Duration(seconds=0.8).to_msg()
            mk.pose = msg.pose
            mk.scale.x, mk.scale.y, mk.scale.z = 0.06, 0.01, 0.01
            mk.color.a, mk.color.r, mk.color.g, mk.color.b = 1.0, 1.0, 0.55, 0.1
            self.marker_pub.publish(mk)

    rclpy.init()
    rclpy.spin(FpPerception())


if __name__ == "__main__":
    main()
