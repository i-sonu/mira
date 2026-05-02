#!/usr/bin/env python3
"""
guided_master.py
────────────────
Extends alt_master.py with GUIDED mode support.

Adds on top of alt_master:
  - set_guided_mode() / set_manual_mode() helpers using pymavlink directly
  - Subscribes to /master/guided_target (geometry_msgs/PoseStamped) to accept
    position targets while in GUIDED mode
  - send_position_target_local_ned() sends SET_POSITION_TARGET_LOCAL_NED to
    the Pixhawk, which ArduSub fuses via EKF3 (fed by ZED via MAVROS odom plugin)
  - send_set_gps_global_origin() sets the EKF origin on startup so EKF3 has
    a reference frame without GPS
  - Everything else (arming, RC overrides, telemetry, emergency kill) is
    identical to alt_master — the Commands topic and all existing subscribers
    are untouched

Operational sequence:
  1. Launch zed_mavros_vio.launch.py  ← MAVROS feeds ZED odom to Pixhawk
  2. Launch guided_master.launch      ← this node owns the MAVLink connection
  3. Wait for "EKF3 IMU0 FUSING ODOMETRY" in QGC
  4. Arm via /master/commands (arm=1) in MANUAL mode as usual
  5. Call /set_guided_mode service  ← switches Pixhawk to GUIDED
  6. Publish geometry_msgs/PoseStamped to /master/guided_target
  7. Call /set_manual_mode to hand back manual control

Frame convention (matches MAVROS odom plugin output):
  SET_POSITION_TARGET_LOCAL_NED uses MAV_FRAME_LOCAL_NED (NED).
  MAVROS converts ZED's ENU odom to NED internally, so the EKF's local frame
  is NED.  Targets published to /master/guided_target should use the same
  ENU convention as ROS (x=forward, y=left, z=up) — this node converts them
  to NED before sending.
"""

import rclpy
from rclpy.node import Node
from pymavlink.dialects.v10 import ardupilotmega
from pymavlink import mavutil
from custom_msgs.msg import Commands, Telemetry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger, Empty
import signal
import sys


def get_name_from_value(value, module=ardupilotmega):
    for name, val in vars(module).items():
        if val == value and name.isupper():
            return name
    return None


class GuidedMaster(Node):
    """
    Extends alt_master with GUIDED mode and position targeting.
    All alt_master behaviour is preserved unchanged.
    """

    emergency_locked = False

    def __init__(self):
        super().__init__("guided_master")

        self.emergency_locked = False
        self.arm_state = False
        self.in_guided_mode = False

        # ── parameters (identical to alt_master) ──────────────────────────────
        self.declare_parameter("initial_mode", "STABILIZE")
        self.declare_parameter("pixhawk_address", "/dev/Pixhawk")

        # EKF origin coordinates — set to your pool/tank GPS location,
        # or leave at 0,0,0 for a bench test.
        self.declare_parameter("ekf_origin_lat", 0.0)
        self.declare_parameter("ekf_origin_lon", 0.0)
        self.declare_parameter("ekf_origin_alt", 0.0)

        self.mode          = self.get_parameter("initial_mode").value
        self.pixhawk_port  = self.get_parameter("pixhawk_address").value
        self.ekf_lat       = self.get_parameter("ekf_origin_lat").value
        self.ekf_lon       = self.get_parameter("ekf_origin_lon").value
        self.ekf_alt       = self.get_parameter("ekf_origin_alt").value

        self.get_logger().info(
            f"pixhawk_address='{self.pixhawk_port}', initial_mode='{self.mode}'"
        )

        # ── MAVLink connection (identical to alt_master) ───────────────────────
        self.master = mavutil.mavlink_connection(self.pixhawk_port, baud=57600)
        self.get_logger().info("MAVLink connection established")

        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 100)

        # ── MAVLink message type handles (identical to alt_master) ─────────────
        self.sys_status_msg   = ardupilotmega.MAVLink_sys_status_message
        self.attitude_msg     = ardupilotmega.MAVLink_attitude_quaternion_message
        self.vfr_hud_msg      = ardupilotmega.MAVLink_vfr_hud_message
        self.depth_msg        = ardupilotmega.MAVLink_scaled_pressure2_message
        self.thruster_pwms_msg = ardupilotmega.MAVLink_servo_output_raw_message
        self.ahrs_msg         = ardupilotmega.MAVLink_ahrs2_message

        # ── ROS 2 publishers (identical to alt_master) ─────────────────────────
        self.telemetry_pub = self.create_publisher(Telemetry, "/master/telemetry", 10)
        self.imu_pub       = self.create_publisher(Imu, "/master/imu_ned", 10)

        # ── ROS 2 subscribers ──────────────────────────────────────────────────
        # Unchanged from alt_master: RC + arming commands
        self.thruster_subs_rov = self.create_subscription(
            Commands, "/master/commands", self.rov_callback, 10
        )

        # NEW: position target for GUIDED mode
        # Publishes geometry_msgs/PoseStamped in ENU (ROS convention):
        #   x = forward (metres from origin)
        #   y = left     (metres from origin)
        #   z = up       (metres from origin, positive = shallower)
        # This node converts ENU → NED before sending SET_POSITION_TARGET_LOCAL_NED.
        self.guided_target_sub = self.create_subscription(
            PoseStamped, "/master/guided_target", self.guided_target_callback, 10
        )

        # ── ROS 2 services ─────────────────────────────────────────────────────
        # Unchanged from alt_master
        self.toggle_kill_srv = self.create_service(
            Trigger, "/toggle_emergency", self.toggle_emergency
        )

        # NEW: switch Pixhawk to GUIDED mode
        self.set_guided_srv = self.create_service(
            Trigger, "/set_guided_mode", self.handle_set_guided_mode
        )

        # NEW: switch Pixhawk back to MANUAL (safe hand-back)
        self.set_manual_srv = self.create_service(
            Trigger, "/set_manual_mode", self.handle_set_manual_mode
        )

        self.channel_ary = [1500] * 8

        self.get_logger().info("Waiting for heartbeat from Pixhawk...")
        self.master.wait_heartbeat()
        self.telem_msg = Telemetry()
        self.get_logger().info("GuidedMaster node initialized")

    # ── GUIDED mode helpers ────────────────────────────────────────────────────

    def set_guided_mode(self):
        """
        Switch Pixhawk to GUIDED mode.

        IMPORTANT (ArduSub bug pre-4.5.7):
          Do NOT arm while already in GUIDED mode — the ROV will immediately
          dive at full thrust.  Arm in MANUAL/STABILIZE first, then call this.
        """
        if "GUIDED" not in self.master.mode_mapping():
            self.get_logger().error("GUIDED mode not found in mode_mapping — check ArduSub firmware")
            return False

        mode_id = self.master.mode_mapping()["GUIDED"]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self.in_guided_mode = True
        self.get_logger().info("Mode switched to GUIDED")
        return True

    def set_mode(self, mode_name: str):
        """Generic mode switch — same logic as alt_master.mode_switch()."""
        if mode_name not in self.master.mode_mapping():
            self.get_logger().error(f"Unknown mode: {mode_name}")
            return False
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self.get_logger().info(f"Mode switched to {mode_name}")
        return True

    def send_set_gps_global_origin(self):
        """
        Send SET_GPS_GLOBAL_ORIGIN so EKF3 has a reference frame without GPS.
        Must be called once after MAVLink connection is established.
        Lat/lon in 1e7 degrees, alt in mm.
        """
        lat_int = int(self.ekf_lat * 1e7)
        lon_int = int(self.ekf_lon * 1e7)
        alt_mm  = int(self.ekf_alt * 1000)

        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,
            lat_int,
            lon_int,
            alt_mm,
        )
        self.get_logger().info(
            f"EKF origin set: lat={self.ekf_lat}, lon={self.ekf_lon}, alt={self.ekf_alt}"
        )

    def send_position_target_local_ned(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
    ):
        """
        Send SET_POSITION_TARGET_LOCAL_NED to Pixhawk.

        Args are in NED (North-East-Down):
          x_m = North   (forward from EKF origin)
          y_m = East    (right from EKF origin)
          z_m = Down    (positive = deeper, negative = shallower)

        The guided_target_callback converts incoming ENU PoseStamped to NED
        before calling this function.

        type_mask: 0b110111111000 = use position only (ignore velocity+accel+yaw).
        """
        TYPE_MASK_POSITION_ONLY = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.master.mav.set_position_target_local_ned_send(
            0,                                          # time_boot_ms (not used)
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,        # NED frame
            TYPE_MASK_POSITION_ONLY,
            x_m, y_m, z_m,                             # position (NED metres)
            vx, vy, vz,                                 # velocity (unused)
            0.0, 0.0, 0.0,                              # acceleration (unused)
            0.0, 0.0,                                   # yaw, yaw_rate (unused)
        )
        self.get_logger().debug(
            f"SET_POSITION_TARGET_LOCAL_NED → N={x_m:.2f} E={y_m:.2f} D={z_m:.2f}"
        )

    # ── Ros 2 callbacks ────────────────────────────────────────────────────────

    def guided_target_callback(self, msg: PoseStamped):
        """
        Receive a position target in ENU (ROS convention) and forward it to
        the Pixhawk as SET_POSITION_TARGET_LOCAL_NED (NED convention).

        ENU → NED:
          x_ned =  y_enu   (East  → North  — only valid if ZED faces vehicle fwd)
          Actually: NED and ENU share the same local origin; the conversion is:
            N =  x_enu  (forward)
            E =  y_enu  (right in NED = left in ENU → negate)
            D = -z_enu  (down in NED = up in ENU → negate)

        For an AUV, z_enu positive = up (shallower), so z_ned = -z_enu = deeper.
        """
        if not self.in_guided_mode:
            self.get_logger().warn(
                "Received guided_target but not in GUIDED mode — ignoring. "
                "Call /set_guided_mode first."
            )
            return

        if self.emergency_locked:
            self.get_logger().warn("Guided target blocked: emergency lock engaged")
            return

        enu_x = msg.pose.position.x
        enu_y = msg.pose.position.y
        enu_z = msg.pose.position.z

        # ENU → NED
        ned_n =  enu_x
        ned_e = -enu_y
        ned_d = -enu_z

        self.send_position_target_local_ned(ned_n, ned_e, ned_d)

    def handle_set_guided_mode(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        if self.emergency_locked:
            response.success = False
            response.message = "Blocked: emergency lock is engaged"
            return response

        if not self.arm_state:
            response.success = False
            response.message = (
                "Arm the vehicle first (in MANUAL/STABILIZE), then switch to GUIDED"
            )
            self.get_logger().warn(response.message)
            return response

        ok = self.set_guided_mode()
        response.success = ok
        response.message = "GUIDED mode set" if ok else "Failed to set GUIDED mode"
        return response

    def handle_set_manual_mode(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        ok = self.set_mode("MANUAL")
        self.in_guided_mode = False
        response.success = ok
        response.message = "MANUAL mode set" if ok else "Failed to set MANUAL mode"
        return response

    # ── everything below is identical to alt_master ───────────────────────────

    def rov_callback(self, msg: Commands):
        if msg.arm == 1 and not self.arm_state:
            if self.emergency_locked:
                self.get_logger().warn("Arm blocked: emergency lock is engaged")
            else:
                self.arm()
                self.arm_state = True
        elif msg.arm == 0 and self.arm_state:
            self.disarm()
            self.arm_state = False

        self.channel_ary[0] = msg.pitch
        self.channel_ary[1] = msg.roll
        self.channel_ary[2] = msg.thrust
        self.channel_ary[3] = msg.yaw
        self.channel_ary[4] = msg.forward
        self.channel_ary[5] = msg.lateral
        self.channel_ary[6] = msg.servo1
        self.channel_ary[7] = msg.servo2

        if self.mode != msg.mode:
            if not self.arm_state:
                self.mode = msg.mode
                self.mode_switch()
            else:
                self.get_logger().warn("Disarm Pixhawk to change modes.")

    def arm(self):
        self.master.wait_heartbeat()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        self.get_logger().info("Arm command sent to Pixhawk")

    def disarm(self):
        self.master.wait_heartbeat()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        self.in_guided_mode = False
        self.get_logger().info("Disarm command sent to Pixhawk")

    def mode_switch(self):
        if self.mode not in self.master.mode_mapping():
            self.get_logger().error(f"Unknown mode: {self.mode}")
            self.get_logger().info(f"Try: {list(self.master.mode_mapping().keys())}")
            exit(1)
        mode_id = self.master.mode_mapping()[self.mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self.get_logger().info(f"Mode changed to: {self.mode}")

    def toggle_emergency(self, request: Trigger.Request, response: Trigger.Response):
        self.emergency_locked = not self.emergency_locked
        if self.emergency_locked:
            if self.arm_state:
                self.disarm()
                self.arm_state = False
            self.in_guided_mode = False
            response.success = True
            response.message = "Emergency lock engaged, disarming"
            self.get_logger().warn(response.message)
        else:
            response.success = True
            response.message = "Emergency lock cleared"
            self.get_logger().info(response.message)
        return response

    def set_rc_channel_pwm(self, id: int, pwm: int):
        if id < 1:
            self.get_logger().warn("Channel does not exist.")
            return
        if id < 9:
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_channel_values,
            )

    def actuate(self):
        # Only send RC overrides in manual modes — not in GUIDED
        if not self.in_guided_mode:
            for i in range(8):
                self.set_rc_channel_pwm(i + 1, int(self.channel_ary[i]))

    def request_message_interval(self, message_id: int, frequency_hz: float):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            1e6 / frequency_hz,
            0, 0, 0, 0, 0,
        )
        response = self.master.recv_match(type="COMMAND_ACK", blocking=True)
        if (
            response
            and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
            and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        ):
            self.get_logger().info(f"Command Accepted for {get_name_from_value(message_id)}")
        else:
            self.get_logger().error(f"Command Failed for {get_name_from_value(message_id)}")

    def telem_publish_func(self):
        self.telem_msg.arm                 = self.arm_state
        self.telem_msg.battery_voltage     = float(self.sys_status_msg.voltage_battery / 1000)
        self.telem_msg.timestamp           = float(self.get_clock().now().to_msg().sec)
        self.telem_msg.internal_pressure   = self.vfr_hud_msg.alt
        self.telem_msg.external_pressure   = float(
            self.depth_msg.press_abs if self.depth_msg is not None else -1
        )
        self.telem_msg.heading             = self.vfr_hud_msg.heading
        self.telem_msg.q1                  = self.attitude_msg.q1
        self.telem_msg.q2                  = self.attitude_msg.q2
        self.telem_msg.q3                  = self.attitude_msg.q3
        self.telem_msg.q4                  = self.attitude_msg.q4
        self.telem_msg.rollspeed           = self.attitude_msg.rollspeed
        self.telem_msg.pitchspeed          = self.attitude_msg.pitchspeed
        self.telem_msg.yawspeed            = self.attitude_msg.yawspeed
        self.telem_msg.roll                = self.ahrs_msg.roll
        self.telem_msg.pitch               = self.ahrs_msg.pitch
        self.telem_msg.yaw                 = self.ahrs_msg.yaw
        self.telem_msg.thruster_pwms[0]    = self.thruster_pwms_msg.servo1_raw
        self.telem_msg.thruster_pwms[1]    = self.thruster_pwms_msg.servo2_raw
        self.telem_msg.thruster_pwms[2]    = self.thruster_pwms_msg.servo3_raw
        self.telem_msg.thruster_pwms[3]    = self.thruster_pwms_msg.servo4_raw
        self.telem_msg.thruster_pwms[4]    = self.thruster_pwms_msg.servo5_raw
        self.telem_msg.thruster_pwms[5]    = self.thruster_pwms_msg.servo6_raw
        self.telem_msg.thruster_pwms[6]    = self.thruster_pwms_msg.servo7_raw
        self.telem_msg.thruster_pwms[7]    = self.thruster_pwms_msg.servo8_raw
        try:
            self.telem_msg.killed = self.emergency_locked
        except Exception:
            pass
        self.telemetry_pub.publish(self.telem_msg)


def main(args=None):
    rclpy.init(args=args)
    obj = GuidedMaster()

    def signal_handler(sig, frame):
        obj.get_logger().info("Signal received, disarming and shutting down...")
        if obj.arm_state:
            obj.disarm()
            obj.arm_state = False
        obj.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # ── request message intervals (identical to alt_master) ───────────────────
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100)
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 100)

    obj.get_logger().info("!!! Is depth sensor connected? Connect to I2C on Pixhawk !!!")

    # ── wait for initial message set (identical to alt_master) ────────────────
    try:
        obj.sys_status_msg    = obj.master.recv_match(type="SYS_STATUS",          blocking=True)
        obj.attitude_msg      = obj.master.recv_match(type="ATTITUDE_QUATERNION", blocking=True)
        obj.vfr_hud_msg       = obj.master.recv_match(type="VFR_HUD",             blocking=True)
        obj.depth_msg         = obj.master.recv_match(type="SCALED_PRESSURE2",    blocking=True)
        obj.thruster_pwms_msg = obj.master.recv_match(type="SERVO_OUTPUT_RAW",    blocking=True)
        obj.ahrs_msg          = obj.master.recv_match(type="AHRS2",               blocking=True)
        obj.get_logger().info("All messages received once")
    except Exception as e:
        obj.get_logger().warn(f"Error receiving initial messages: {e}")
        exit()

    # ── send EKF origin (new — required for GUIDED without GPS) ───────────────
    obj.send_set_gps_global_origin()

    # ── main loop (identical to alt_master) ───────────────────────────────────
    try:
        while rclpy.ok():
            obj.actuate()
            try:
                obj.sys_status_msg    = obj.master.recv_match(type="SYS_STATUS",          blocking=True)
                obj.attitude_msg      = obj.master.recv_match(type="ATTITUDE_QUATERNION", blocking=True)
                obj.vfr_hud_msg       = obj.master.recv_match(type="VFR_HUD",             blocking=True)
                obj.depth_msg         = obj.master.recv_match(type="SCALED_PRESSURE2",    blocking=True)
                obj.thruster_pwms_msg = obj.master.recv_match(type="SERVO_OUTPUT_RAW",    blocking=True)
            except Exception as e:
                obj.get_logger().warn(f"Error receiving message: {e}")
                continue

            obj.telem_publish_func()
            rclpy.spin_once(obj, timeout_sec=0.1)

    except KeyboardInterrupt:
        obj.get_logger().info("KeyboardInterrupt, disarming and shutting down...")
        if obj.arm_state:
            obj.disarm()
            obj.arm_state = False
    finally:
        obj.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()