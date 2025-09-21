#!/usr/bin/env python3
# -- coding: utf-8 --

import cv2 as cv
import numpy as np
import yaml
import glob
import time
import os
import re
import math
import sys
from collections import deque
try:
    import serial
except ImportError:
    serial = None

# Configuration settings
CONFIG = {
    "TURN_DEG": 90.0,  # Corner turn angle
    "TURN_TOL_DEG": 10.0,  # Turn tolerance
    "PILLAR_TURN_DEG": 90.0,  # Pillar sidestep turn angle
    "FLIP_TURN_DEG": 180.0,  # Flip turn angle
    "FLIP_FWD_HOLD_S": 1.5,  # Forward hold before flip
    "FLIP_LEFT_SIGN": 1.0,  # +1 for left turn on RED pillar
    "CORNER_TURN_MAX_S": 2.3,  # Max time for corner turn
    "PILLAR_TURN_MAX_S": 2.0,  # Max time for pillar turn
    "MAX_TURNS": 12,  # Max number of turns
    "CENTER_DEG": 90,  # Servo center position
    "LEFT_LIMIT": 65,  # Servo right-most limit
    "RIGHT_LIMIT": 120,  # Servo left-most limit
    "DRIVE_SPEED": 26,  # Default drive speed (PWM)
    "SEE_COLOR_SPEED": 22,  # Speed when color detected
    "TURN_SPEED": 28,  # Turn speed
    "PILLAR_TURN_SPEED": 25,  # Pillar turn speed
    "PILLAR_FWD_SPEED": 26,  # Pillar forward speed
    "PILLAR_BACK_SPEED": 26,  # Pillar backward speed
    "BACK_LEFT_SERVO_DEG": 65,  # Servo angle for BLUE reverse turn
    "BACK_RIGHT_SERVO_DEG": 120,  # Servo angle for ORANGE reverse turn
    "FRONT_TURN_CM": 25.0,  # Distance to trigger corner turn
    "BACK_STOP_CM": 30.0,  # Distance to stop backward centering
    "PILLAR_ACCEPT_CM": 50.0,  # Pillar detection distance
    "PILLAR_APPROACH_FRONT_CM": 6.0,  # Pillar approach distance
    "PILLAR_BACK_TO_FRONT_CM": 13.0,  # Pillar back-off distance
    "COLOR_CONFIRM_FRAMES": 2,  # Frames to confirm color
    "MIN_PIX_COLOR": 50,  # Min pixels for color detection
    "PILLAR_MIN_PIX": 80,  # Min pixels for pillar detection
    "COLOR_ROI_WIDTH_FRAC": 0.28,  # Color ROI width fraction
    "COLOR_ROI_HEIGHT_FRAC": 0.20,  # Color ROI height fraction
    "PILLAR_ROI_WIDTH_FRAC": 0.60,  # Pillar ROI width fraction
    "PILLAR_ROI_HEIGHT_FRAC": 0.55,  # Pillar ROI height fraction
    "PILLAR_DIST_K": 4000.0,  # Pillar distance constant
    "RED_LINE_X_FRAC": 0.150,  # Red pillar gate x-position
    "GREEN_LINE_X_FRAC": 0.850,  # Green pillar gate x-position
    "STEER_KP_DEG_PER_DEG": 0.835,  # Proportional gain for steering
    "STEER_MAX_DEG": 8.0,  # Max steering offset
    "STEER_SLEW_DEG": 20,  # Max steering change rate
    "WALL_AVOID_ON_CM": 7.5,  # Wall avoidance trigger distance
    "WALL_AVOID_OFF_CM": 8.5,  # Wall avoidance release distance
    "WALL_AVOID_HOLD_S": 0.35,  # Wall avoidance hold time
    "WALL_AVOID_KP_DEG_PER_CM": 1.8,  # Wall avoidance gain
    "WALL_AVOID_SPEED": 24,  # Wall avoidance speed
    "FRAME_W": 640,  # Frame width
    "FRAME_H": 480,  # Frame height
    "ROBOT_PORT": "/dev/ttyUSB0",  # Serial port
    "ROBOT_BAUD": 115200,  # Baud rate
    "STALE_LINK_SEC": 2.5,  # Link timeout
    "PING_PERIOD_SEC": 0.7,  # Ping interval
    "US_MAX_CM": 300.0,  # Max ultrasonic distance
}

# Camera management class
class Camera:
    def _init_(self):
        self.width = CONFIG["FRAME_W"]
        self.height = CONFIG["FRAME_H"]
        self.cap = self._open_camera()

    def _gstreamer_pipeline(self):
        # GStreamer pipeline for video capture
        return (
            f"nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, "
            f"format=NV12, framerate=30/1 ! nvvidconv ! "
            f"video/x-raw, width={self.width}, height={self.height}, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
        )

    def _open_camera(self):
        # Try opening camera with GStreamer or V4L2
        try:
            cap = cv.VideoCapture(self._gstreamer_pipeline(), cv.CAP_GSTREAMER)
            if cap.isOpened():
                return cap
        except Exception:
            pass
        try:
            cap = cv.VideoCapture(0, cv.CAP_V4L2)
            if cap.isOpened():
                return cap
        except Exception:
            pass
        print("Error: Camera failed to open.")
        return None

    def read(self):
        # Read frame from camera
        if not self.cap:
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            return False, frame
        ok, frame = self.cap.read()
        if not ok:
            cv.putText(frame, "Frame read failed", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return ok, frame

    def release(self):
        # Release camera
        if self.cap:
            self.cap.release()

# Serial communication class
class SerialInterface:
    def _init_(self):
        self.serial = None
        self.port = CONFIG["ROBOT_PORT"]
        self.baud = CONFIG["ROBOT_BAUD"]
        self.last_ping = 0.0
        self.last_tlm = 0.0
        self.last_pong = 0.0
        self.rx_buf = b""
        self.tlm_pattern = re.compile(
            r"^TLM,.*yaw=([\-0-9\.]+),state=([0-9]),steer=([\-0-9\.]+),speed=([0-9]+),"
            r"dF=([\-0-9]+),dL=([\-0-9]+),dR=([\-0-9]+),dB=([\-0-9]+)"
        )
        self._open_serial()

    def _open_serial(self):
        # Open serial connection
        if serial is None:
            print("Warning: Serial library not available.")
            return
        try:
            self.serial = serial.Serial(self.port, baudrate=self.baud, timeout=0.01, write_timeout=0.2)
            time.sleep(0.3)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            print(f"Serial opened on {self.port}")
        except Exception as e:
            print(f"Warning: Serial failed to open: {e}")
            self.serial = None

    def write(self, data):
        # Write data to serial
        if not self.serial:
            self._open_serial()
            if not self.serial:
                return False
        try:
            self.serial.write(data)
            return True
        except Exception as e:
            print(f"Error writing to serial: {e}")
            self.serial = None
            return False

    def read_telemetry(self):
        # Read and parse telemetry
        if not self.serial:
            return None, False
        try:
            data = self.serial.read(256)
            if not data:
                return None, False
            self.rx_buf += data
            yaw, us_data, armed = None, None, False
            while b"\n" in self.rx_buf:
                line, self.rx_buf = self.rx_buf.split(b"\n", 1)
                s = line.decode("utf-8", "ignore").strip()
                if s == "PONG" or s == "READY":
                    self.last_pong = time.time()
                    continue
                if s == "ARMED":
                    self.last_pong = time.time()
                    armed = True
                    continue
                match = self.tlm_pattern.match(s)
                if match:
                    self.last_tlm = time.time()
                    yaw = float(match.group(1))
                    us_data = {
                        "dF": int(match.group(5)),
                        "dL": int(match.group(6)),
                        "dR": int(match.group(7)),
                        "dB": int(match.group(8)),
                    }
            return {"yaw": yaw, "us": us_data}, armed
        except Exception as e:
            print(f"Error reading serial: {e}")
            self.serial = None
            return None, False

    def ping(self, now):
        # Send periodic ping
        if now - self.last_ping >= CONFIG["PING_PERIOD_SEC"]:
            self.write(b"PING\n")
            self.last_ping = now

# Robot controller class
class RobotController:
    def _init_(self):
        self.camera = Camera()
        self.serial = SerialInterface()
        self.yaml_config, self.yaml_path = self._load_yaml()
        self.state = "WAIT_START"
        self.yaw = 0.0
        self.start_yaw = 0.0
        self.corridor_idx = 0
        self.orientation_offset = 0.0
        self.turn_count = 0
        self.color_queue = deque(maxlen=CONFIG["COLOR_CONFIRM_FRAMES"])
        self.pillar_queue = deque(maxlen=CONFIG["COLOR_CONFIRM_FRAMES"])
        self.pillar_active = False
        self.pillar_state = "IDLE"
        self.pillar_color = None
        self.last_steer_cmd = None
        self.err_ema = 0.0
        self.avoid_active = False
        self.avoid_side = None
        self.first_pillar_after_turn = {1: None, 2: None, 3: None, 4: None}
        self.last_pillar_before_turn4 = None
        self.flip_scheduled = False
        self.flip_color = None
        self.us_ema = {"dF": -1.0, "dL": -1.0, "dR": -1.0, "dB": -1.0}
        print(f"Loaded YAML config: {self.yaml_path}")

    def _load_yaml(self):
        # Load latest YAML config for color ranges
        cands = sorted(glob.glob("config/vision_*.yaml"))
        if not cands:
            raise FileNotFoundError("No config file in ./config")
        path = cands[-1]
        with open(path, "r") as f:
            return yaml.safe_load(f), path

    def _wrap180(self, angle):
        # Wrap angle to [-180, 180]
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def _deg_to_norm(self, angle_deg):
        # Convert servo angle to normalized value
        angle_deg = max(CONFIG["LEFT_LIMIT"], min(CONFIG["RIGHT_LIMIT"], float(angle_deg)))
        if angle_deg >= CONFIG["CENTER_DEG"]:
            return (angle_deg - CONFIG["CENTER_DEG"]) / abs(CONFIG["RIGHT_LIMIT"] - CONFIG["CENTER_DEG"])
        return (angle_deg - CONFIG["CENTER_DEG"]) / abs(CONFIG["CENTER_DEG"] - CONFIG["LEFT_LIMIT"])

    def _slew_limit(self, prev, target, max_delta):
        # Limit rate of change for steering
        if prev is None:
            return target
        return max(prev - max_delta, min(prev + max_delta, target))

    def _mask_hsv(self, hsv, ranges):
        # Create mask for HSV color range
        if isinstance(ranges, dict) and "low" in ranges and "high" in ranges:
            low = np.array(ranges["low"], dtype=np.uint8)
            high = np.array(ranges["high"], dtype=np.uint8)
            return cv.inRange(hsv, low, high)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in (ranges if isinstance(ranges, list) else []):
            if "low" in r and "high" in r:
                m = cv.inRange(hsv, np.array(r["low"], dtype=np.uint8), np.array(r["high"], dtype=np.uint8))
                mask = cv.bitwise_or(mask, m)
        return mask

    def _detect_colors(self, frame):
        # Detect BLUE/ORANGE lines and RED/GREEN pillars
        h, w = frame.shape[:2]
        # Color ROI for lines
        roi_w = int(w * CONFIG["COLOR_ROI_WIDTH_FRAC"])
        roi_h = int(h * CONFIG["COLOR_ROI_HEIGHT_FRAC"])
        cx0, cy0 = (w - roi_w) // 2, h - roi_h
        roi_color = frame[cy0:cy0 + roi_h, cx0:cx0 + roi_w]
        roi_color = cv.GaussianBlur(roi_color, (3, 3), 0)
        hsv_color = cv.cvtColor(roi_color, cv.COLOR_BGR2HSV)
        m_blue = self._mask_hsv(hsv_color, self.yaml_config.get("BLUE", {}))
        m_orange = self._mask_hsv(hsv_color, self.yaml_config.get("ORANGE", {}))
        pix_blue = np.count_nonzero(m_blue)
        pix_orange = np.count_nonzero(m_orange)
        color = "BLUE" if pix_blue >= CONFIG["MIN_PIX_COLOR"] else "ORANGE" if pix_orange >= CONFIG["MIN_PIX_COLOR"] else None

        # Pillar ROI
        p_roi_w = int(w * CONFIG["PILLAR_ROI_WIDTH_FRAC"])
        p_roi_h = int(h * CONFIG["PILLAR_ROI_HEIGHT_FRAC"])
        px0, py0 = (w - p_roi_w) // 2, h - p_roi_h - 20
        roi_pillar = frame[py0:py0 + p_roi_h, px0:px0 + p_roi_w]
        roi_pillar = cv.GaussianBlur(roi_pillar, (5, 5), 0)
        hsv_pillar = cv.cvtColor(roi_pillar, cv.COLOR_BGR2HSV)
        m_red = self._mask_hsv(hsv_pillar, self.yaml_config.get("RED", {}))
        m_green = self._mask_hsv(hsv_pillar, self.yaml_config.get("GREEN", {}))
        red_blob = max(cv.findContours(m_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0], key=cv.contourArea, default=None)
        green_blob = max(cv.findContours(m_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0], key=cv.contourArea, default=None)
        pillar = None
        pillar_dist = None
        if red_blob and cv.contourArea(red_blob) >= CONFIG["PILLAR_MIN_PIX"]:
            pillar = "RED"
            x, y, w, h = cv.boundingRect(red_blob)
            pillar_dist = CONFIG["PILLAR_DIST_K"] / max(1.0, float(h))
        elif green_blob and cv.contourArea(green_blob) >= CONFIG["PILLAR_MIN_PIX"]:
            pillar = "GREEN"
            x, y, w, h = cv.boundingRect(green_blob)
            pillar_dist = CONFIG["PILLAR_DIST_K"] / max(1.0, float(h))
        return color, pillar, pillar_dist

    def _compute_steering(self, yaw):
        # Compute steering angle for yaw centering
        yaw_ref = self._wrap180(self.start_yaw + self.corridor_idx * CONFIG["TURN_DEG"] + self.orientation_offset)
        error = self._wrap180(yaw_ref - yaw)
        self.err_ema = 0.75 * self.err_ema + 0.25 * error
        steer_deg = CONFIG["CENTER_DEG"] + CONFIG["STEER_KP_DEG_PER_DEG"] * self.err_ema
        steer_deg = max(CONFIG["LEFT_LIMIT"], min(CONFIG["RIGHT_LIMIT"], steer_deg))
        steer_deg = self._slew_limit(self.last_steer_cmd, steer_deg, CONFIG["STEER_SLEW_DEG"])
        self.last_steer_cmd = steer_deg
        return steer_deg

    def run(self):
        # Main control loop
        cv.namedWindow("robot", cv.WINDOW_AUTOSIZE)
        last_tx = 0.0
        corner_color = None
        corner_dir_latch = None
        avoid_until = 0.0
        pillar_turn_target = None
        pillar_turn_start = 0.0
        flip_wait_until = 0.0
        flip_target = None
        corner_turn_start = 0.0
        print("Ready. Keys: S=start, Q=quit, X=reset")

        while True:
            now = time.time()
            self.serial.ping(now)
            tlm, armed = self.serial.read_telemetry()
            if tlm:
                self.yaw = tlm["yaw"] or self.yaw
                for k, v in tlm["us"].items():
                    if v >= 0:
                        self.us_ema[k] = 0.65 * self.us_ema[k] + 0.35 * v if self.us_ema[k] >= 0 else v

            if self.state == "WAIT_START" and armed:
                self.start_yaw = self.yaw
                self.corridor_idx = 0
                self.orientation_offset = 0.0
                self.state = "RUN"
                print(f"Started. Initial yaw: {self.yaw:.1f}")

            ok, frame = self.camera.read()
            if not ok:
                print("Error: Frame read failed.")
                break
            h, w = frame.shape[:2]
            color, pillar, pillar_dist = self._detect_colors(frame)

            if self.state == "RUN" and not self.pillar_active and color:
                self.color_queue.append(color)
                if len(self.color_queue) == CONFIG["COLOR_CONFIRM_FRAMES"]:
                    corner_color = self.color_queue[0]
                    self.color_queue.clear()
                    if self.turn_count == 0:
                        corner_dir_latch = -1 if corner_color == "BLUE" else 1
                        print(f"Latched corner direction: {corner_color} -> {'LEFT' if corner_dir_latch == -1 else 'RIGHT'}")
                    target_idx = self.corridor_idx + (corner_dir_latch or (1 if corner_color == "ORANGE" else -1))
                    if self.turn_count >= 8 and self.last_pillar_before_turn4 == "RED":
                        target_idx = self.corridor_idx - (corner_dir_latch or (1 if corner_color == "ORANGE" else -1))
                    yaw_target = self._wrap180(self.start_yaw + target_idx * CONFIG["TURN_DEG"] + self.orientation_offset)
                    if tlm and tlm["us"]["dF"] >= 0 and tlm["us"]["dF"] < CONFIG["FRONT_TURN_CM"]:
                        self.state = "CORNER_TURNING"
                        corner_turn_start = now
                        print(f"Corner detected: {corner_color}, turning to yaw {yaw_target:.1f}")

            if self.state == "RUN" and not self.pillar_active and pillar and pillar_dist <= CONFIG["PILLAR_ACCEPT_CM"]:
                self.pillar_queue.append(pillar)
                if len(self.pillar_queue) == CONFIG["COLOR_CONFIRM_FRAMES"]:
                    self.pillar_active = True
                    self.pillar_color = self.pillar_queue[0]
                    self.pillar_state = "TURN"
                    yaw_ref = self._wrap180(self.start_yaw + self.corridor_idx * CONFIG["TURN_DEG"] + self.orientation_offset)
                    sign = 1 if self.pillar_color == "RED" else -1
                    pillar_turn_target = self._wrap180(yaw_ref + sign * CONFIG["PILLAR_TURN_DEG"])
                    pillar_turn_start = now
                    self.serial.write(f"TURN_ABS,{pillar_turn_target:.1f},{CONFIG['PILLAR_TURN_SPEED']}\n".encode())
                    print(f"Pillar {self.pillar_color} at {pillar_dist:.1f}cm, turning {'RIGHT' if sign > 0 else 'LEFT'}")

            if self.state == "RUN":
                dL, dR = self.us_ema.get("dL", -1), self.us_ema.get("dR", -1)
                if dL >= 0 and dL <= CONFIG["WALL_AVOID_ON_CM"]:
                    self.avoid_side = "L"
                    self.avoid_active = True
                    avoid_until = now + CONFIG["WALL_AVOID_HOLD_S"]
                elif dR >= 0 and dR <= CONFIG["WALL_AVOID_ON_CM"]:
                    self.avoid_side = "R"
                    self.avoid_active = True
                    avoid_until = now + CONFIG["WALL_AVOID_HOLD_S"]
                elif self.avoid_active and now >= avoid_until:
                    self.avoid_active = False
                    self.avoid_side = None

                steer_deg = self._compute_steering(self.yaw)
                speed = CONFIG["SEE_COLOR_SPEED"] if corner_color else CONFIG["DRIVE_SPEED"]
                if self.avoid_active:
                    d_side = dL if self.avoid_side == "L" else dR
                    steer_deg += (-1 if self.avoid_side == "L" else 1) * CONFIG["WALL_AVOID_KP_DEG_PER_CM"] * (CONFIG["WALL_AVOID_ON_CM"] - d_side)
                    speed = CONFIG["WALL_AVOID_SPEED"]
                if now - last_tx > 0.008:
                    self.serial.write(f"CENTER,{self._deg_to_norm(steer_deg):.3f},{speed}\n".encode())
                    last_tx = now

            if self.pillar_active:
                if self.pillar_state == "TURN" and (abs(self._wrap180(pillar_turn_target - self.yaw)) <= CONFIG["TURN_TOL_DEG"] or now - pillar_turn_start >= CONFIG["PILLAR_TURN_MAX_S"]):
                    self.serial.write(b"STOP\n")
                    self.pillar_state = "FWD"
                elif self.pillar_state == "FWD" and tlm and tlm["us"]["dF"] >= 0 and tlm["us"]["dF"] < CONFIG["PILLAR_APPROACH_FRONT_CM"]:
                    self.pillar_state = "BACK"
                elif self.pillar_state == "BACK" and tlm and tlm["us"]["dF"] >= CONFIG["PILLAR_BACK_TO_FRONT_CM"]:
                    self.pillar_state = "RETURN"
                    yaw_ref = self._wrap180(self.start_yaw + self.corridor_idx * CONFIG["TURN_DEG"] + self.orientation_offset)
                    self.serial.write(f"TURN_ABS,{yaw_ref:.1f},{CONFIG['PILLAR_TURN_SPEED']}\n".encode())
                elif self.pillar_state == "RETURN" and (abs(self._wrap180(yaw_ref - self.yaw)) <= CONFIG["TURN_TOL_DEG"] or now - pillar_turn_start >= CONFIG["PILLAR_TURN_MAX_S"]):
                    self.serial.write(b"STOP\n")
                    self.pillar_active = False
                    self.pillar_state = "IDLE"
                    if self.turn_count == 8 and self.last_pillar_before_turn4 == "RED":
                        self.flip_scheduled = True
                        self.flip_color = self.pillar_color
                        flip_wait_until = now + CONFIG["FLIP_FWD_HOLD_S"]
                        self.state = "FLIP_WAIT"

            if self.state == "FLIP_WAIT" and now >= flip_wait_until:
                self.state = "FLIP_TURN"
                yaw_ref = self._wrap180(self.start_yaw + self.corridor_idx * CONFIG["TURN_DEG"] + self.orientation_offset)
                sign = CONFIG["FLIP_LEFT_SIGN"] if self.flip_color == "RED" else -CONFIG["FLIP_LEFT_SIGN"]
                flip_target = self._wrap180(yaw_ref + sign * CONFIG["FLIP_TURN_DEG"])
                self.serial.write(f"TURN_ABS,{flip_target:.1f},{CONFIG['PILLAR_TURN_SPEED']}\n".encode())

            if self.state == "FLIP_TURN" and abs(self._wrap180(flip_target - self.yaw)) <= CONFIG["TURN_TOL_DEG"]:
                self.serial.write(b"STOP\n")
                self.orientation_offset = self._wrap180(self.orientation_offset + (CONFIG["FLIP_LEFT_SIGN"] if self.flip_color == "RED" else -CONFIG["FLIP_LEFT_SIGN"]) * CONFIG["FLIP_TURN_DEG"])
                self.state = "RUN"
                self.flip_scheduled = False

            cv.imshow("robot", frame)
            key = cv.waitKey(1) & 0xFF
            if key == ord("q"):
                self.serial.write(b"STOP\n")
                break
            elif key == ord("s"):
                self.state = "RUN"
                self.serial.write(b"START\n")
                self.start_yaw = self.yaw
                print("Started.")
            elif key == ord("x"):
                self.serial.write(b"STOP\n")
                self.serial._open_serial()
                self.state = "WAIT_START"
                self.pillar_active = False
                self.flip_scheduled = False
                print("Reset.")

        self.camera.release()
        cv.destroyAllWindows()

if _name_ == "_main_":
    try:
        RobotController().run()
    except KeyboardInterrupt:
        print("Exiting.")
        sys.exit(0)