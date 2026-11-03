#!/usr/bin/env python3
# Author: Steven (https://github.com/sng516157-web) — 2026-11-03
"""Classical perception for the SC plug, v1: detect the plug in /plug_cam and
publish its pose in gazebo_world on /perception/plug_pose.

Method: the table is the only brown surface in the scene; the plug and its
stands are grey. Segment low color-skew pixels (|r-b| small) below the
horizon, take the blob centroid, project the camera ray onto the plug's
resting plane (z from config), and recover yaw from the blob's principal axis.
(X/Y land within ~1 cm; yaw is rough — the weld-time self-calibration
measures the true pose at grasp anyway.)

Test offline on a saved frame:  python3 perception_node.py --test frame.png
"""

import math
import sys

import numpy as np

# plug_cam in sim/insertion.sdf (keep in sync!)
CAM_POSE = (0.34, -0.30, 1.05, 0.0, 0.89, 2.16)  # x y z roll pitch yaw
IMG_W, IMG_H = 640, 480
HFOV = 0.9
PLUG_Z = 0.785  # resting height of the plug body frame on the stands


def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


FX = IMG_W / (2 * math.tan(HFOV / 2))
VFOV = 2 * math.atan(math.tan(HFOV / 2) * IMG_H / IMG_W)
FY = IMG_H / (2 * math.tan(VFOV / 2))


def blue_mask(rgb):
    """Scene segmentation: grey objects (plug + stands) on the brown table,
    below the image horizon. The prong-end blue was too dim under this
    camera's angle; the grey/brown contrast is the reliable one."""
    r = rgb[:, :, 0].astype(int)
    g = rgb[:, :, 1].astype(int)
    b = rgb[:, :, 2].astype(int)
    h = np.indices(r.shape)[0]
    return (np.abs(r - b) < 25) & (np.abs(g - b) < 20) & (r < 235) & (h > 120)


def estimate(rgb, cam_pose=CAM_POSE, plug_z=PLUG_Z):
    """RGB array -> (x, y, z, yaw) in gazebo_world, or None if nothing seen."""
    mask = blue_mask(rgb)
    ys, xs = np.nonzero(mask)
    if len(xs) < 30:
        return None

    cx, cy = xs.mean(), ys.mean()
    # Principal axis of the blob in image space (plug's long axis direction).
    coords = np.stack([xs - cx, ys - cy])
    cov = coords @ coords.T / len(xs)
    eigvals, eigvecs = np.linalg.eigh(cov)
    axis_img = eigvecs[:, np.argmax(eigvals)]  # unit vector along the blob

    # Camera ray through the centroid, projected onto the plug's rest plane.
    cam = np.array(cam_pose[:3])
    R = rotation_matrix(*cam_pose[3:])
    ray_cam = np.array([1.0, -(cx - IMG_W / 2) / FX, -(cy - IMG_H / 2) / FY])
    ray = R @ ray_cam
    if abs(ray[2]) < 1e-6:
        return None
    t = (plug_z - cam[2]) / ray[2]
    point = cam + t * ray

    # Find the yaw whose image-plane axis direction matches the blob's.
    # The plug's long axis is its x axis; on a horizontal plane a yaw fully
    # determines the imaged direction.
    def imaged_axis_dir(yaw):
        a0 = R @ (np.array([math.cos(yaw), math.sin(yaw), 0.0]))
        # project onto the image plane (drop forward component, use -y,-z)
        v = np.array([-a0[1], -a0[2]])
        n = np.linalg.norm(v)
        return v / n if n > 1e-9 else v

    best_yaw, best_err = 0.0, 1e9
    for yaw in np.arange(0, math.pi, 0.01):
        d = imaged_axis_dir(yaw)
        err = 1.0 - abs(float(d @ axis_img))  # direction is sign-ambiguous here
        if err < best_err:
            best_err, best_yaw = err, yaw

    # Sign: the blue housings sit at the PRONG end, biasing the blob centroid
    # toward the prongs — so the prong side is the SHORTER centroid-to-edge
    # distance along the axis.
    proj = coords.T @ axis_img
    d_max, d_min = proj.max(), proj.min()
    prong_sign = 1.0 if abs(d_min) > abs(d_max) else -1.0

    yaw = best_yaw if prong_sign > 0 else best_yaw + math.pi
    return (float(point[0]), float(point[1]), plug_z, float(yaw % (2 * math.pi)))


def main_test(path):
    from PIL import Image as PILImage
    rgb = np.array(PILImage.open(path).convert("RGB"))
    est = estimate(rgb)
    if est is None:
        print("plug not found in frame")
        return
    x, y, z, yaw = est
    print(f"estimated plug pose: x={x:.4f} y={y:.4f} z={z:.3f} yaw={yaw:.2f} rad "
          f"({math.degrees(yaw):.0f} deg)")
    print("ground truth spawn:  x=0.2200 y=-0.1200 z=0.785 yaw=0.00 rad")


def main_ros():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image

    class PerceptionNode(Node):
        def __init__(self):
            super().__init__("plug_perception")
            self.declare_parameter("image_topic", "/plug_cam")
            self.declare_parameter("plug_z", PLUG_Z)
            self.pub = self.create_publisher(PoseStamped, "/perception/plug_pose", 10)
            self.create_subscription(
                Image, self.get_parameter("image_topic").value, self.cb, 10)
            self.get_logger().info("plug perception node up (classical v1)")

        def cb(self, msg):
            rgb = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            est = estimate(rgb, plug_z=self.get_parameter("plug_z").value)
            if est is None:
                return
            x, y, z, yaw = est
            out = PoseStamped()
            out.header.frame_id = "gazebo_world"
            out.header.stamp = self.get_clock().now().to_msg()
            out.pose.position.x = x
            out.pose.position.y = y
            out.pose.position.z = z
            out.pose.orientation.z = math.sin(yaw / 2)
            out.pose.orientation.w = math.cos(yaw / 2)
            self.pub.publish(out)

    rclpy.init()
    rclpy.spin(PerceptionNode())


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--test":
        main_test(sys.argv[2])
    else:
        main_ros()
