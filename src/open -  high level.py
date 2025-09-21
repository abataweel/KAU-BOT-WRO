#!/usr/bin/env python3
# -- coding: utf-8 --
"""
Jetson side controller for robot navigation with camera and serial communication.
Hotkeys:
  S -> Send START command
  X -> Reset Arduino with backoff
  Q -> STOP and quit
"""

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

class Config:
    """Configuration parameters loaded from environment or defaults."""
    def _init_(self):
        self.params = {
            # Turn and geometry
            'turn_deg': float(os.environ.get('TURN_DEG', '85.0')),
            'turn_trigger_tolerance_deg': float(os.environ.get('TURN_TRIGGER_TOL_DEG', '80.0')),
            'center_turn_step_deg': float(os.environ.get('CENTER_TURN_STEP_DEG', '90.0')),

            # Run and stop conditions
            'max_turns': int(os.environ.get('MAX_TURNS', '12')),
            'stop_at_end_cm': float(os.environ.get('STOP_AT_END_CM', '140.0')),

            # Servo geometry
            'center_deg': int(os.environ.get('CENTER_DEG', '90')),
            'left_limit': int(os.environ.get('LEFT_LIMIT', '60')),
            'right_limit': int(os.environ.get('RIGHT_LIMIT', '110')),
            'servo_reversed': int(os.environ.get('SERVO_REVERSED', '1')),

            # Drive speeds
            'drive_speed': int(os.environ.get('DRIVE_SPEED', '43')),
            'turn_speed': int(os.environ.get('TURN_SPEED', '27')),
            'post_turn_speed': int(os.environ.get('POST_TURN_SPEED', '25')),
            'post_turn_duration': float(os.environ.get('POST_TURN_SEC', '1.5')),
            'see_color_speed': int(os.environ.get('SEE_COLOR_SPEED', '27')),
            'min_speed': int(os.environ.get('MIN_SPEED', '0')),

            # Wall proximity slowdown
            'wall_slow_side_on_cm': float(os.environ.get('WALL_SLOW_SIDE_ON_CM', '8.0')),
            'wall_slow_side_off_cm': float(os.environ.get('WALL_SLOW_SIDE_OFF_CM', '9.0')),
            'side_slowdown_factor': float(os.environ.get('SIDE_SLOWDOWN_FACTOR', '0.55')),
            'wall_slow_front_on_cm': float(os.environ.get('WALL_SLOW_FRONT_ON_CM', '45.0')),
            'front_slowdown_factor': float(os.environ.get('FRONT_SLOW_FRONT_ON_CM', '0.550')),
            'wall_slow_min_speed': int(os.environ.get('WALL_SLOW_MIN_SPEED', '21')),

            # Wall steering
            'wall_steer_gain': float(os.environ.get('WALL_STEER_GAIN', '.3')),
            'wall_steer_max_add_deg': float(os.environ.get('WALL_STEER_MAX_ADD_DEG', '0.80')),
            'wall_steer_expand_center_clamp': int(os.environ.get('WALL_STEER_EXPAND_CENTER_CLAMP', '1')),
            'wall_steer_center_min': int(os.environ.get('WALL_STEER_CENTER_MIN', '80')),
            'wall_steer_center_max': int(os.environ.get('WALL_STEER_CENTER_MAX', '100')),

            # Wall avoidance
            'wall_avoid_on_cm': float(os.environ.get('WALL_AVOID_ON_CM', '11.5')),
            'wall_avoid_off_cm': float(os.environ.get('WALL_AVOID_OFF_CM', '12.5')),
            'wall_avoid_max_bias_deg': float(os.environ.get('WALL_AVOID_MAX_BIAS_DEG', '25.0')),
            'wall_avoid_gain': float(os.environ.get('WALL_AVOID_GAIN', '0.65')),
            'wall_avoid_speed': int(os.environ.get('WALL_AVOID_SPEED', '24')),
            'wall_avoid_timeout_s': float(os.environ.get('WALL_AVOID_TIMEOUT_S', '1.2')),

            # Contact failsafe
            'contact_on_cm': float(os.environ.get('CONTACT_ON_CM', '12.0')),
            'contact_treat_invalid_as_on': int(os.environ.get('CONTACT_TREAT_INVALID_AS_ON', '1')),
            'contact_steer_deg': float(os.environ.get('CONTACT_STEER_DEG', '14.0')),
            'contact_hold_s': float(os.environ.get('CONTACT_HOLD_S', '0.3')),
            'contact_speed_factor': float(os.environ.get('CONTACT_SPEED_FACTOR', '0.6')),

            # Turn gating
            'front_turn_threshold_cm': float(os.environ.get('FRONT_TURN_THRESH_CM', '35.0')),
            'turn_use_side_gate': int(os.environ.get('TURN_USE_SIDE_GATE', '1')),
            'turn_side_open_cm': float(os.environ.get('TURN_SIDE_OPEN_CM', '120.0')),
            'turn_side_logic': os.environ.get('TURN_SIDE_LOGIC', 'OR').upper(),
            'turn_gate_combine': os.environ.get('TURN_GATE_COMBINE', 'OR').upper(),

            # Color detection
            'color_confirm_frames': int(os.environ.get('COLOR_CONFIRM_FRAMES', '1')),
            'detect_cooldown_s': float(os.environ.get('DETECT_COOLDOWN_S', '2.5')),
            'min_pixel_color': int(os.environ.get('MIN_PIX_COLOR', '1')),
            'color_erode_iterations': int(os.environ.get('COLOR_ERODE_IT', '0')),
            'color_dilate_iterations': int(os.environ.get('COLOR_DILATE_IT', '1')),
            'color_roi_width_frac': float(os.environ.get('COLOR_ROI_WIDTH_FRAC', '0.25')),
            'color_roi_height_frac': float(os.environ.get('COLOR_ROI_HEIGHT_FRAC', '0.20')),
            'roi_scale_after_first_turn': float(os.environ.get('ROI_SCALE_AFTER_FIRST_TURN', '1.3')),
            'color_roi_after_turns': int(os.environ.get('COLOR_ROI_AFTER_TURNS', '0')),
            'color_pick_mode': 'maxpix',
            'color_turn_map': {'BLUE': 'LEFT', 'ORANGE': 'RIGHT'},
            'saturation_boost': float(os.environ.get('SAT_BOOST', '1.15')),
            'blur_kernel_size': int(os.environ.get('BLUR_KSIZE', '3')),
            'color_suspend_sec': float(os.environ.get('COLOR_SUSPEND_SEC', '2.5')),

            # Color re-arm
            'color_rearm_front_cm': float(os.environ.get('COLOR_REARM_FRONT_CM', '80.0')),
            'color_rearm_delay_s': float(os.environ.get('COLOR_REARM_DELAY_S', '0.6')),
            'disable_detect_during_turn': True,

            # Yaw centering
            'steer_kp_deg_per_deg': float(os.environ.get('STEER_KP_DEG_PER_DEG', '8.90')),
            'steer_exponent': float(os.environ.get('STEER_EXP', '1.0')),
            'steer_dead_error_deg': float(os.environ.get('STEER_DEAD_ERR_DEG', '0.02')),
            'steer_output_deadband_deg': float(os.environ.get('STEER_OUT_DB_DEG', '.2')),
            'steer_max_deg': float(os.environ.get('STEER_MAX_DEG', '5.0')),
            'error_filter_alpha': float(os.environ.get('ERR_FILTER_ALPHA', '1.75')),
            'steer_slew_deg': int(os.environ.get('STEER_SLEW_DEG', '20')),
            'center_steer_min': 70,
            'center_steer_max': 110,

            # Ultrasonic
            'us_max_cm': float(os.environ.get('US_MAX_CM', '300.0')),
            'us_ema_alpha': float(os.environ.get('US_EMA_ALPHA', '0.35')),
            'us_repel_on_cm': float(os.environ.get('US_REPEL_ON_CM', '18.0')),
            'us_repel_off_cm': float(os.environ.get('US_REPEL_OFF_CM', '22.0')),
            'us_repel_gain': float(os.environ.get('US_REPEL_K', '0.06')),
            'us_repel_steer_boost_deg': float(os.environ.get('US_REPEL_STEER_BOOST_DEG', '6.0')),
            'us_repel_expand_center_clamp': int(os.environ.get('US_REPEL_EXPND_CENTER_CLAMP', '1')),
            'us_repel_center_min': int(os.environ.get('US_REPEL_CENTER_MIN', '80')),
            'us_repel_center_max': int(os.environ.get('US_REPEL_CENTER_MAX', '100')),

            # Camera
            'allow_no_camera': int(os.environ.get('ALLOW_NO_CAMERA', '1')),
            'camera_backend': os.environ.get('CAMERA_BACKEND', 'AUTO').upper(),
            'frame_width': int(os.environ.get('FRAME_W', '640')),
            'frame_height': int(os.environ.get('FRAME_H', '480')),

            # Final run
            'final_burst_duration': float(os.environ.get('FINAL_BURST_S', '1.0')),
            'final_us_delay_s': float(os.environ.get('FINAL_US_DELAY_S', '.9')),
            'reverse_supported': int(os.environ.get('REVERSE_SUPPORTED', '0')),
            'reverse_speed': int(os.environ.get('REVERSE_SPEED', '30')),
            'reverse_max_duration': float(os.environ.get('REVERSE_MAX_S', '2.0')),

            # Serial
            'robot_port': os.environ.get('ROBOT_PORT', '/dev/ttyUSB0'),
            'robot_baud': int(os.environ.get('ROBOT_BAUD', '115200')),
            'stale_telemetry_sec': float(os.environ.get('STALE_TLM_SEC', '1.2')),
            'stale_link_sec': float(os.environ.get('STALE_LINK_SEC', '2.5')),
            'ping_period_sec': float(os.environ.get('PING_PERIOD_SEC', '0.7')),
            'reset_backoff_sec': float(os.environ.get('RESET_BACKOFF_SEC', '4.0')),
            'ard_reset_method': os.environ.get('ARD_RESET_METHOD', 'AUTO').upper(),
            'ard_reset_touch_delay_s': float(os.environ.get('ARD_RESET_TOUCH_DELAY_S', '0.8')),
            'ard_reset_dtr_pulse_s': float(os.environ.get('ARD_RESET_DTR_PULSE_S', '0.12')),
        }
        self.servo_dir = -1 if self.params['servo_reversed'] else 1
        self.span_left = abs(self.params['center_deg'] - self.params['left_limit'])
        self.span_right = abs(self.params['right_limit'] - self.params['center_deg'])

class RobotController:
    """Main controller for robot navigation and communication."""
    def _init_(self, config: Config):
        self.config = config
        self.serial_port = None
        self.camera = None
        self.dummy_camera = False
        self.state = 'WAIT_START'
        self.yaw_start = None
        self.yaw_ref = 0.0
        self.turn_count = 0
        self.last_turn_target = None
        self.last_turn_time = -1e9
        self.first_turn_dir = None
        self.current_turn_dir = None
        self.turn_requested = False
        self.detect_enabled = True
        self.strict_latch_active = False
        self.rearm_started_time = -1e9
        self.rearm_after_turn_time = -1.0
        self.color_queue = deque(maxlen=config.params['color_confirm_frames'])
        self.turn_latched_dir = None
        self.see_color_slow_active = False
        self.see_color_consumed = False
        self.side_slow_active = False
        self.contact_active = False
        self.contact_dir = None
        self.contact_until = -1.0
        self.color_pause_until = -1.0
        self.err_ema = 0.0
        self.last_steer = None
        self.last_tx_time = 0.0
        self.repel_left_on = False
        self.repel_right_on = False
        self.final_burst_start = -1.0
        self.final_back_start = -1.0
        self.post_boost_until = -1.0
        self.final_eval_not_before = -1.0
        self.wall_avoid_state = 'IDLE'
        self.wall_avoid_start_time = -1.0
        self.wall_avoid_side = None
        self.last_yaw = 0.0
        self.last_us = {'dF': -1, 'dL': -1, 'dR': -1}
        self.us_ema = {'dF': -1.0, 'dL': -1.0, 'dR': -1.0}
        self.arduino_armed = False
        self.last_tlm_time = 0.0
        self.last_pong_time = 0.0
        self.last_ping_time = 0.0
        self.last_reset_time = -1e9
        self.rx_buffer = b""
        self.tlm_regex = re.compile(r"^TLM,.*yaw=([\-0-9\.]+),state=([0-9]),steer=([\-0-9\.]+),speed=([0-9]+),dF=([\-0-9]+),dL=([\-0-9]+),dR=([\-0-9]+)")

    def initialize_serial(self):
        """Initialize serial communication with the Arduino."""
        if serial is None:
            print("PySerial not available.")
            return
        try:
            self.serial_port = serial.Serial(
                self.config.params['robot_port'],
                baudrate=self.config.params['robot_baud'],
                timeout=0.01,
                write_timeout=0.2,
                dsrdtr=False
            )
            time.sleep(0.35)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            print(f"Serial OK on {self.config.params['robot_port']}")
        except Exception as e:
            print(f"WARNING: Serial open failed -> {e}")
            self.serial_port = None

    def safe_write(self, data: bytes) -> bool:
        """Safely write to the serial port with reconnection logic."""
        if serial is None or self.serial_port is None:
            self.initialize_serial()
            if self.serial_port is None:
                return False
        try:
            self.serial_port.write(data)
            return True
        except Exception as e:
            print(f"Serial write error: {e}")
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
            self.initialize_serial()
            if self.serial_port:
                try:
                    self.serial_port.write(data)
                    return True
                except Exception as e2:
                    print(f"Write retry failed: {e2}")
            return False

    def reset_arduino(self, method: str = "AUTO") -> bool:
        """Reset the Arduino with the specified method."""
        now = time.time()
        if now - self.last_reset_time < self.config.params['reset_backoff_sec']:
            return False
        self.last_reset_time = now
        method = method.upper()

        def touch_1200():
            try:
                print("[RESET] 1200-bps touch...")
                tmp = serial.Serial(self.config.params['robot_port'], 1200, timeout=0.05)
                time.sleep(0.05)
                tmp.setDTR(False)
                tmp.flush()
                tmp.close()
                time.sleep(self.config.params['ard_reset_touch_delay_s'])
                return True
            except Exception as e:
                print(f"[RESET] 1200-bps touch failed: {e}")
                return False

        def dtr_pulse():
            try:
                print("[RESET] DTR pulse...")
                s = serial.Serial(self.config.params['robot_port'], baudrate=self.config.params['robot_baud'], timeout=0.05, dsrdtr=False)
                s.dtr = False
                time.sleep(self.config.params['ard_reset_dtr_pulse_s'])
                s.dtr = True
                time.sleep(0.25)
                s.close()
                return True
            except Exception as e:
                print(f"[RESET] DTR pulse failed: {e}")
                return False

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
            time.sleep(0.1)

        success = False
        if method in ('AUTO', 'TOUCH1200'):
            success = touch_1200()
        if method == 'AUTO' and not success:
            success = dtr_pulse()
        elif method == 'DTR':
            success = dtr_pulse()

        self.initialize_serial()
        print("[RESET] Done." if success else "[RESET] Failed.")
        return success

    def is_link_alive(self, now: float) -> bool:
        """Check if the serial link is alive."""
        return (now - max(self.last_tlm_time, self.last_pong_time)) < self.config.params['stale_link_sec']

    def send_ping(self, now: float):
        """Send a ping command if the period has elapsed."""
        if now - self.last_ping_time >= self.config.params['ping_period_sec']:
            self.safe_write(b"PING\n")
            self.last_ping_time = now

    def read_telemetry(self):
        """Read and parse telemetry from the Arduino."""
        if self.serial_port is None:
            return
        try:
            data = self.serial_port.read(256)
            if not data:
                return
            self.rx_buffer += data
            while b"\n" in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split(b"\n", 1)
                line_str = line.decode('utf-8', 'ignore').strip()
                if line_str == 'PONG':
                    self.last_pong_time = time.time()
                    continue
                if line_str == 'READY':
                    self.last_pong_time = time.time()
                    continue
                if line_str == 'ARMED':
                    self.arduino_armed = True
                    self.last_pong_time = time.time()
                    continue
                match = self.tlm_regex.match(line_str)
                if match:
                    self.last_yaw = float(match.group(1))
                    self.last_tlm_time = time.time()
                    dF = int(match.group(5))
                    dL = int(match.group(6))
                    dR = int(match.group(7))
                    alpha = self.config.params['us_ema_alpha']
                    for key, value in (('dF', dF), ('dL', dL), ('dR', dR)):
                        value = float(value)
                        if value < 0:
                            self.last_us[key] = -1
                            continue
                        if self.us_ema[key] < 0:
                            self.us_ema[key] = value
                        else:
                            self.us_ema[key] = (1 - alpha) * self.us_ema[key] + alpha * value
                        self.last_us[key] = int(self.us_ema[key])
        except Exception as e:
            print(f"Serial read error: {e}")
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.initialize_serial()

    def initialize_camera(self):
        """Initialize the camera with the specified backend."""
        backend = self.config.params['camera_backend']
        width, height = self.config.params['frame_width'], self.config.params['frame_height']

        def try_gstreamer():
            pipeline = (
                f"nvarguscamerasrc sensor-id=0 ! "
                f"video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=30/1 ! "
                f"nvvidconv flip-method=0 ! "
                f"video/x-raw, width={width}, height={height}, format=(string)BGRx ! "
               "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=1"
            )
            try:
                cap = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)
                return cap if cap.isOpened() else None
            except Exception:
                return None

        def try_v4l2():
            try:
                cap = cv.VideoCapture(0, cv.CAP_V4L2)
                return cap if cap.isOpened() else None
            except Exception:
                return None

        self.camera = try_gstreamer() if backend == 'GST'