#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink.dialects.v10 import ardupilotmega
from pymavlink import mavutil
from custom_msgs.msg import CommandsNormalized, Telemetry
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from rclpy.utilities import remove_ros_args
import signal
import sys

# Channel mapping:
# 1   Pitch
# 2   Roll
# 3   Throttle
# 4   Yaw
# 5   Forward
# 6   Lateral

_PWM_NEUTRAL = 1500
_PWM_SCALE = 400  # -1 → 1100, 0 → 1500, +1 → 1900


def _to_pwm(value: float) -> int:
    return int(_PWM_NEUTRAL + value * _PWM_SCALE)


def get_name_from_value(value, module=ardupilotmega):
    for name, val in vars(module).items():
        if val == value and name.isupper():
            return name
    return None


class PixhawkMasterNormalized(Node):
    """MAVLink bridge accepting normalized (-1 to 1) commands on /master/commands_normalized."""

    def __init__(self):
        super().__init__("pymav_master_normalized")
        self.emergency_locked = False
        self.arm_state = False

        self.declare_parameter("initial_mode", "STABILIZE")
        self.declare_parameter("pixhawk_address", "/dev/Pixhawk")

        self.mode = self.get_parameter("initial_mode").value
        self.pixhawk_port = self.get_parameter("pixhawk_address").value

        self.get_logger().info(f"pixhawk_address='{self.pixhawk_port}', initial_mode='{self.mode}'")

        self.master = mavutil.mavlink_connection(self.pixhawk_port, baud=57600)
        self.get_logger().info("MAVLink connection established")

        self.get_logger().info("Waiting for heartbeat from Pixhawk...")
        self.master.wait_heartbeat()

        self.get_logger().info("Requesting HEARTBEAT")
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 100)

        self.sys_status_msg = ardupilotmega.MAVLink_sys_status_message
        self.attitude_msg = ardupilotmega.MAVLink_attitude_quaternion_message
        self.vfr_hud_msg = ardupilotmega.MAVLink_vfr_hud_message
        self.depth_msg = ardupilotmega.MAVLink_scaled_pressure2_message
        self.thruster_pwms_msg = ardupilotmega.MAVLink_servo_output_raw_message
        self.attitude_euler = ardupilotmega.MAVLink_attitude_message

        self.telemetry_pub = self.create_publisher(Telemetry, "/master/telemetry", 10)
        self.imu_pub = self.create_publisher(Imu, "/master/imu_ned", 10)
        self.create_subscription(
            CommandsNormalized, "/master/commands_normalized", self.cmd_callback, 10
        )
        self.create_service(Trigger, "/toggle_emergency", self.toggle_emergency)

        self.channel_ary = [_PWM_NEUTRAL] * 8
        self.telem_msg = Telemetry()
        self.get_logger().info("PixhawkMasterNormalized node initialized")

    def cmd_callback(self, msg: CommandsNormalized):
        if msg.arm == 1 and not self.arm_state:
            if self.emergency_locked:
                self.get_logger().warn("Arm blocked: emergency lock is engaged")
            else:
                self.arm()
                self.arm_state = True
        elif msg.arm == 0 and self.arm_state:
            self.disarm()
            self.arm_state = False

        self.channel_ary[0] = _to_pwm(msg.pitch)
        self.channel_ary[1] = _to_pwm(msg.roll)
        self.channel_ary[2] = _to_pwm(msg.thrust)
        self.channel_ary[3] = _to_pwm(msg.yaw)
        self.channel_ary[4] = _to_pwm(msg.forward)
        self.channel_ary[5] = _to_pwm(msg.lateral)
        self.channel_ary[6] = _to_pwm(msg.servo1)
        self.channel_ary[7] = _to_pwm(msg.servo2)

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
            rc_channel_values = [65535] * 8
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_channel_values,
            )

    def actuate(self):
        for i in range(8):
            self.set_rc_channel_pwm(i + 1, self.channel_ary[i])

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
        self.telem_msg.arm = self.arm_state
        self.telem_msg.battery_voltage = float(self.sys_status_msg.voltage_battery / 1000)
        self.telem_msg.timestamp = float(self.get_clock().now().to_msg().sec)
        self.telem_msg.internal_pressure = self.vfr_hud_msg.alt
        self.telem_msg.external_pressure = float(
            self.depth_msg.press_abs if self.depth_msg is not None else -1
        )
        self.telem_msg.heading = self.vfr_hud_msg.heading
        self.telem_msg.q1 = self.attitude_msg.q1
        self.telem_msg.q2 = self.attitude_msg.q2
        self.telem_msg.q3 = self.attitude_msg.q3
        self.telem_msg.q4 = self.attitude_msg.q4
        self.telem_msg.rollspeed = self.attitude_msg.rollspeed
        self.telem_msg.pitchspeed = self.attitude_msg.pitchspeed
        self.telem_msg.yawspeed = self.attitude_msg.yawspeed
        self.telem_msg.roll = self.attitude_euler.roll
        self.telem_msg.pitch = self.attitude_euler.pitch
        self.telem_msg.yaw = self.attitude_euler.yaw
        self.telem_msg.thruster_pwms[0] = self.thruster_pwms_msg.servo1_raw
        self.telem_msg.thruster_pwms[1] = self.thruster_pwms_msg.servo2_raw
        self.telem_msg.thruster_pwms[2] = self.thruster_pwms_msg.servo3_raw
        self.telem_msg.thruster_pwms[3] = self.thruster_pwms_msg.servo4_raw
        self.telem_msg.thruster_pwms[4] = self.thruster_pwms_msg.servo5_raw
        self.telem_msg.thruster_pwms[5] = self.thruster_pwms_msg.servo6_raw
        self.telem_msg.thruster_pwms[6] = self.thruster_pwms_msg.servo7_raw
        self.telem_msg.thruster_pwms[7] = self.thruster_pwms_msg.servo8_raw
        try:
            self.telem_msg.killed = self.emergency_locked
        except Exception:
            pass
        self.telemetry_pub.publish(self.telem_msg)


def main(args=None):
    rclpy.init(args=args)
    obj = PixhawkMasterNormalized()

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

    obj.get_logger().info("Requesting HEARTBEAT")
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 100)
    obj.get_logger().info("Requesting SYS_STATUS")
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 100)
    obj.get_logger().info("Requesting ATTITUDE_QUATERNION")
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100)
    obj.get_logger().info("Requesting SCALED_PRESSURE2")
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 100)
    obj.get_logger().info("Requesting VFR_HUD")
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 100)
    obj.get_logger().info("Requesting SERVO_OUTPUT_RAW")
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100)
    obj.get_logger().info("Requesting ATTITUDE")
    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100)

    obj.get_logger().info("!!! Is depth sensor connected? Connect it to I2C port on the Pixhawk !!!")

    try:
        obj.get_logger().info("Waiting for SYS_STATUS")
        obj.sys_status_msg = obj.master.recv_match(type="SYS_STATUS", blocking=True)
        obj.get_logger().info("Waiting for ATTITUDE_QUATERNION")
        obj.attitude_msg = obj.master.recv_match(type="ATTITUDE_QUATERNION", blocking=True)
        obj.get_logger().info("Waiting for VFR_HUD")
        obj.vfr_hud_msg = obj.master.recv_match(type="VFR_HUD", blocking=True)
        obj.get_logger().info("Waiting for SCALED_PRESSURE2")
        obj.depth_msg = obj.master.recv_match(type="SCALED_PRESSURE2", blocking=True)
        obj.get_logger().info("Waiting for SERVO_OUTPUT_RAW")
        obj.thruster_pwms_msg = obj.master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True)
        obj.get_logger().info("Waiting for ATTITUDE")
        obj.attitude_euler = obj.master.recv_match(type="ATTITUDE", blocking=True)
        obj.get_logger().info("All messages received once")
    except Exception as e:
        obj.get_logger().warn(f"Error receiving all messages: {e}")
        exit()

    try:
        while rclpy.ok():
            obj.actuate()
            try:
                obj.sys_status_msg = obj.master.recv_match(type="SYS_STATUS", blocking=True)
                obj.attitude_msg = obj.master.recv_match(type="ATTITUDE_QUATERNION", blocking=True)
                obj.attitude_euler = obj.master.recv_match(type="ATTITUDE", blocking=True)
                obj.vfr_hud_msg = obj.master.recv_match(type="VFR_HUD", blocking=True)
                obj.depth_msg = obj.master.recv_match(type="SCALED_PRESSURE2", blocking=True)
                obj.thruster_pwms_msg = obj.master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True)
            except Exception as e:
                obj.get_logger().warn(f"Error receiving message: {e}")
                continue
            obj.telem_publish_func()
            rclpy.spin_once(obj, timeout_sec=0.1)
    except KeyboardInterrupt:
        obj.get_logger().info("KeyboardInterrupt received, disarming and shutting down...")
        if obj.arm_state:
            obj.disarm()
            obj.arm_state = False
    finally:
        obj.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
