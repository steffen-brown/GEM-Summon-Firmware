#!/usr/bin/env python3
#================================================================
# File name: lane_detection.py                                                                  
# Description: learning‑based lane detection module with live OpenCV preview                     
# Author: Siddharth Anand (orig.) / ChatGPT patch May‑2025                                        
#================================================================

from __future__ import print_function

# Python Headers
import os
import cv2
import csv
import math
import time
import torch
import numpy as np
from collections import deque
from numpy import linalg as la
import scipy.signal as signal
from cv_bridge import CvBridge, CvBridgeError

from filters import OnlineFilter

# ROS Headers
import rospy
from nav_msgs.msg import Path
import alvinxy.alvinxy as axy  # Import AlvinXY transformation module

# GEM Sensor Headers
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from geometry_msgs.msg import PoseStamped
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt

###############################################################################
# Lane Detection Node                                                         #
###############################################################################

class LaneNetDetector:
    """Main class for lane detection using YOLOPv2 neural network."""

    def __init__(self, path_to_weights='../../../weights/yolopv2.pt'):
        # ───────────────────────── Safety checks ────────────────────────────
        if not os.path.exists(path_to_weights):
            raise FileNotFoundError(f"Model weights not found at {path_to_weights}")

        # Frame buffering for batch processing to increase efficiency
        self.frame_buffer = []
        self.buffer_size = 4  # Process 4 frames at once for better throughput

        # Initialize ROS node
        rospy.init_node('lane_detection_node', anonymous=True)
        rospy.on_shutdown(self.cleanup)  # make sure windows close cleanly

        # OpenCV live‑preview window (resizable)
        cv2.namedWindow("Lane Preview", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Lane Preview", 800, 480)

        # Image processing utilities
        self.bridge = CvBridge()
        self.prev_left_boundary = None
        self.prev_waypoints = None
        self.endgoal = None

        self.estimated_lane_width_pixels = 400  # start‑up guess (px)
        self.last_left_x = None
        self.last_right_x = None
        self.lane_width_history = deque(maxlen=15)

        # ───────────────────────── Deep Learning Model ──────────────────────
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = torch.jit.load(path_to_weights)
        self.half = self.device != 'cpu'
        if self.half:
            self.model.half()
        self.model.to(self.device).eval()

        # ───────────────────────── Camera / control params ──────────────────
        self.Focal_Length = 800  # px
        self.Real_Height_SS = 0.75  # m
        self.Brake_Distance = 5  # m
        self.Brake_Duration = 3  # s

        # ───────────────────────── ROS comms ────────────────────────────────
        # Subscribers
        self.sub_image = rospy.Subscriber(
            '/zed2/zed_node/left/image_rect_color', Image, self.img_callback, queue_size=1)

        # Publishers
        self.pub_contrasted_image = rospy.Publisher(
            'LANE_DETECTION/contrasted_image', Image, queue_size=1)
        self.pub_annotated = rospy.Publisher('lane_detection/annotate', Image, queue_size=1)
        self.pub_waypoints = rospy.Publisher('lane_detection/waypoints', Path, queue_size=1)
        self.pub_endgoal = rospy.Publisher('LANE_DETECTION/endgoal', PoseStamped, queue_size=1)

    # ───────────────────────── ROS image callback ───────────────────────────
    def img_callback(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

            # ——— Image enhancement (yellow emphasis) ———
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
            yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
            non_yellow_mask = cv2.bitwise_not(yellow_mask)
            img_no_yellow = cv2.bitwise_and(img, img, mask=non_yellow_mask)
            gray = cv2.cvtColor(img_no_yellow, cv2.COLOR_BGR2GRAY)
            threshold = 180
            mask = gray >= threshold
            dimmed = (gray * 0.5).astype(np.uint8)
            dimmed[mask] = gray[mask]
            contrasted = cv2.cvtColor(dimmed, cv2.COLOR_GRAY2BGR)

            self.pub_contrasted_image.publish(
                self.bridge.cv2_to_imgmsg(contrasted, "bgr8"))

            # ——— Batch inference prep ———
            tensor = self.preprocess_frame(contrasted)
            self.frame_buffer.append((contrasted, tensor))

            if len(self.frame_buffer) >= self.buffer_size:
                originals, tensors = zip(*self.frame_buffer)
                batch = torch.stack(tensors).to(self.device)
                self.frame_buffer.clear()
                with torch.no_grad():
                    [pred, anchor_grid], seg, ll = self.model(batch)
                for i, orig in enumerate(originals):
                    annotated = self.detect_lanes(seg[i], ll[i], orig)
                    self.pub_annotated.publish(
                        self.bridge.cv2_to_imgmsg(annotated, "bgr8"))

                    # Live preview
                    cv2.imshow("Lane Preview", annotated)
                    cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)

    # ───────────────────────── Pre‑processing util ──────────────────────────
    def preprocess_frame(self, img):
        img_resized, _, _ = self.letterbox(img, new_shape=(384, 640))
        img_resized = img_resized[:, :, ::-1].transpose(2, 0, 1)  # BGR‑>RGB, HWC‑>CHW
        tensor = torch.from_numpy(np.ascontiguousarray(img_resized))
        tensor = tensor.half() if self.half else tensor.float()
        tensor /= 255.0
        return tensor

    # ───────────────────────── Lane‑processing pipeline ─────────────────────
    def detect_lanes(self, seg, ll, img):
        da_seg_mask = driving_area_mask(seg)
        ll_seg_mask = lane_line_mask(ll, threshold=0.2)

        waypoints, left_boundary, right_boundary = self.generate_waypoints(ll_seg_mask)
        annotated = self.draw_waypoints(img.copy(), waypoints, left_boundary, right_boundary)
        self.publish_waypoints(waypoints)
        return annotated

    # ---------------------------------------------------------------------
    #  CURVE‑ROBUST generate_waypoints()  (returns both lane edges)         
    # ---------------------------------------------------------------------
    def generate_waypoints(self, lane_mask):
        path = Path(); path.header.frame_id = "map"
        h, w = lane_mask.shape
        step = max(6, int(0.02 * h))
        left_raw, right_raw = [], []

        # 1 — scan mask bottom→top
        for y in range(h - 1, -1, -step):
            xs = np.where(lane_mask[y, :] > 0)[0]
            if xs.size > 1:
                x_l, x_r = xs[0], xs[-1]
                left_raw.append((x_l, y)); right_raw.append((x_r, y))
            elif xs.size == 1:
                x = xs[0]
                if x < w // 2:
                    left_raw.append((x, y)); right_raw.append(None)
                else:
                    left_raw.append(None); right_raw.append((x, y))
            else:
                left_raw.append(None); right_raw.append(None)

        # 2 — tidy edges
        left = self.filter_continuous_boundary(left_raw)
        right = self.filter_continuous_boundary(right_raw)

        # 3 — build centreline & update metrics
        for lb, rb in zip(left, right):
            if lb is not None and rb is not None:
                lane_w = rb[0] - lb[0]
                self.lane_width_history.append(lane_w)
            elif lb is not None:
                lane_w = np.median(self.lane_width_history) if self.lane_width_history else self.estimated_lane_width_pixels
                rb = (lb[0] + int(lane_w), lb[1])
            elif rb is not None:
                lane_w = np.median(self.lane_width_history) if self.lane_width_history else self.estimated_lane_width_pixels
                lb = (rb[0] - int(lane_w), rb[1])
            else:
                continue

            self.estimated_lane_width_pixels = max(0.35 * w, 0.7 * self.estimated_lane_width_pixels + 0.3 * lane_w)
            self.last_left_x, self.last_right_x = lb[0], rb[0]

            x_c = (lb[0] + rb[0]) // 2; y_c = lb[1]
            pose = PoseStamped(); pose.pose.position.x = x_c; pose.pose.position.y = y_c
            path.poses.append(pose)

        # 4 — crop & end‑goal
        path.poses = path.poses[:12]
        if path.poses:
            xs = [p.pose.position.x for p in path.poses]
            ys = [p.pose.position.y for p in path.poses]
            self.endgoal = PoseStamped(header=path.header)
            self.endgoal.pose.position.x = float(np.mean(xs))
            self.endgoal.pose.position.y = float(np.mean(ys))
        else:
            self.endgoal = None

        return path, left, right

    # ───────────────────────── Util: continuity filter ──────────────────────
    def filter_continuous_boundary(self, boundary):
        max_gap = 60
        continuous = []; prev = None
        for pt in boundary:
            if pt is not None:
                if prev is None or abs(pt[0]-prev[0]) <= max_gap:
                    continuous.append(pt); prev = pt
                else:
                    continuous.append(None); prev = None
            else:
                continuous.append(None); prev = None
        return continuous

    # ───────────────────────── Publish ROS topics ───────────────────────────
    def publish_waypoints(self, waypoints):
        self.pub_waypoints.publish(waypoints)
        if self.endgoal is not None:
            self.pub_endgoal.publish(self.endgoal)

    # ───────────────────────── Drawing helpers ──────────────────────────────
    def draw_waypoints(self, img, waypoints, left_boundary, right_boundary):
        # centre points – yellow
        for p in waypoints.poses:
            cv2.circle(img, (int(p.pose.position.x), int(p.pose.position.y)), 5, (0,255,255), -1)
        # left edge – blue
        for lb in left_boundary:
            if lb is not None:
                cv2.circle(img, lb, 3, (255,0,0), -1)
        # right edge – green
        for rb in right_boundary:
            if rb is not None:
                cv2.circle(img, rb, 3, (0,255,0), -1)
        # end‑goal – red
        if self.endgoal is not None:
            ex = int(self.endgoal.pose.position.x); ey = int(self.endgoal.pose.position.y)
            cv2.circle(img, (ex, ey), 10, (0,0,255), -1)
            cv2.putText(img, "Endgoal", (ex+15, ey), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        return img

    # ───────────────────────── Letterbox util ───────────────────────────────
    def letterbox(self, img, new_shape=(384, 640), color=(114,114,114)):
        shape = img.shape[:2]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)
        r = min(new_shape[0]/shape[0], new_shape[1]/shape[1])
        new_unpad = (int(round(shape[1]*r)), int(round(shape[0]*r)))
        dw, dh = new_shape[1]-new_unpad[0], new_shape[0]-new_unpad[1]
        dw, dh = dw/2, dh/2
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh-0.1)), int(round(dh+0.1))
        left, right = int(round(dw-0.1)), int(round(dw+0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return img, (r,r), (dw,dh)

    # ───────────────────────── Cleanup on shutdown ──────────────────────────
    def cleanup(self):
        cv2.destroyAllWindows()

###############################################################################
# Stand‑alone helpers                                                         #
###############################################################################

def driving_area_mask(seg):
    if len(seg.shape) == 4:
        da_predict = seg[:, :, 12:372, :]
    elif len(seg.shape) == 3:
        seg = seg.unsqueeze(0)
        da_predict = seg[:, :, 12:372, :]
    else:
        raise ValueError(f"Unexpected tensor shape: {seg.shape}")
    da_seg_mask = torch.nn.functional.interpolate(da_predict, scale_factor=2, mode='bilinear', align_corners=False)
    _, da_seg_mask = torch.max(da_seg_mask, 1)
    return da_seg_mask.int().squeeze().cpu().numpy()


def lane_line_mask(ll, threshold):
    if len(ll.shape) == 4:
        ll_predict = ll[:, :, 12:372, :]
    elif len(ll.shape) == 3:
        ll = ll.unsqueeze(0)
        ll_predict = ll[:, :, 12:372, :]
    else:
        raise ValueError(f"Unexpected tensor shape: {ll.shape}")
    ll_seg_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=2, mode='bilinear', align_corners=False)
    ll_seg_mask = (ll_seg_mask > threshold).int().squeeze(1)
    ll_seg_mask = ll_seg_mask.squeeze().cpu().numpy().astype(np.uint8)
    ll_seg_mask = cv2.dilate(ll_seg_mask, np.ones((2,2), np.uint8), iterations=1)
    return ll_seg_mask

###############################################################################
# Main entry                                                                  #
###############################################################################

if __name__ == "__main__":
    try:
        detector = LaneNetDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
