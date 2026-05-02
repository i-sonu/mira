"""
zed_mavros_vio.launch.py
ZED visual odometry  →  MAVROS  →  ArduSub EKF3  →  GUIDED mode

  ONE-TIME ARDUSUB PARAMETER SETUP (set via QGC, then reboot)
  AHRS_EKF_TYPE  = 3    switch to EKF3
  EK3_ENABLE     = 1
  EK3_SRC1_POSXY = 6    horizontal position  → ExternalNAV (ZED)
  EK3_SRC1_VELXY = 6    horizontal velocity  → ExternalNAV (ZED)
  EK3_SRC1_POSZ  = 1    vertical position    → Baro (depth sensor)
                         set to 6 if you want ZED to handle depth too
  EK3_SRC1_VELZ  = 6    vertical velocity    → ExternalNAV (ZED)
  EK3_SRC1_YAW   = 6    yaw                  → ExternalNAV (ZED IMU)
                         set to 1 to keep the onboard compass for yaw
  VISO_TYPE      = 1    enable visual odometry frontend
  GPS1_TYPE      = 0    disable GPS
  ARMING_CHECK   = 0    disable GPS arming check (or use ARMING_SKIPCHK)
  → REBOOT after saving

  After reboot, QGC Messages tab should show:
    "EKF3 IMU0 STARTED RELATIVE AIDING"
    "EKF3 IMU0 FUSING ODOMETRY"

  GUIDED MODE OPERATIONAL SEQUENCE
  1. Launch this file — wait for MAVROS to connect
  2. Publish EKF origin (see ekf_origin_publisher_node below)
  3. Wait for "EKF3 FUSING ODOMETRY" in QGC
       (known ArduSub bug pre-4.5.7: arming in GUIDED dives immediately)
  5. Switch to GUIDED:
       ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
           "{custom_mode: 'GUIDED'}"
  6. Send position targets to /mavros/setpoint_position/local
       (geometry_msgs/PoseStamped, frame_id=map, z is NED: negative=down)

  USAGE
  ros2 launch mira2_control_master zed_mavros_vio.launch.py
  ros2 launch mira2_control_master zed_mavros_vio.launch.py \
      pixhawk_address:=/dev/ttyACM0
  ros2 launch mira2_control_master zed_mavros_vio.launch.py \
      gcs_url:=udp://@192.168.2.100:14550

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pixhawk_address_arg = DeclareLaunchArgument(
        "pixhawk_address",
        default_value="/dev/Pixhawk",
        description="Serial device for the Pixhawk (e.g. /dev/Pixhawk, /dev/ttyACM0)",
    )

    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url",
        default_value="",
        description=(
            "GCS UDP URL for telemetry forwarding "
            "(e.g. udp://@192.168.2.100:14550). Leave empty to disable."
        ),
    )

    zed_odom_topic_arg = DeclareLaunchArgument(
        "zed_odom_topic",
        default_value="/zed/zed_node/odom",
        description="nav_msgs/Odometry topic from the ZED ROS 2 wrapper node",
    )

    pixhawk_address = LaunchConfiguration("pixhawk_address")
    gcs_url = LaunchConfiguration("gcs_url")
    zed_odom_topic = LaunchConfiguration("zed_odom_topic")

    fcu_url = PythonExpression(["'serial://' + '", pixhawk_address, "' + ':57600'"])

    # ── static TF ─────────────────────────────────────────────────────────────
    # Attaches ZED's ENU `odom` frame under the world `map` frame so the
    # MAVROS odom plugin's tf-validation passes.
    # If the ZED is mounted at a heading offset relative to the vehicle's
    # forward axis, set the yaw argument (radians) to match. For example,
    # if the ZED faces left (90° port), set yaw to "1.5708".
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        # tx  ty  tz  yaw  pitch  roll  parent  child
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # ── MAVROS node ───────────────────────────────────────────────────────────
    # Connects to ArduSub over serial at the same baud as alt_master.py.
    # The `odom` plugin (mavros_extras) is loaded by default — it reads from
    # ~/odometry/out and sends MAVLink ODOMETRY (id 331) to the FCU.
    # MAVLink 2 is required; ODOMETRY (id 331) is a MAVLink 2-only message.
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        namespace="mavros",
        output="screen",
        parameters=[
            {
                "fcu_url": fcu_url,
                "gcs_url": gcs_url,
                "fcu_protocol": "v2.0",
                # ArduSub default target IDs
                "target_system_id": 1,
                "target_component_id": 1,
                "odometry/fcu/odom_parent_id_des": "odom",
                "odometry/fcu/odom_child_id_des": "base_link",
            }
        ],
        remappings=[
            ("odometry/out", zed_odom_topic),
        ],
    )

    ekf_origin_node = Node(
        package="mavros",
        executable="vision_pose_estimate",  # placeholder — replace with your
        name="ekf_origin_publisher",  # own node or pymavlink script if
        namespace="mavros",  # this executable doesn't suit.
        output="screen",
        # ── OR use this pymavlink one-liner from a shell after launch: ────────
        # python3 -c "
        # from pymavlink import mavutil
        # m = mavutil.mavlink_connection('/dev/Pixhawk', baud=57600)
        # m.wait_heartbeat()
        # m.mav.set_gps_global_origin_send(m.target_system, 0, 0, 0)
        # print('EKF origin sent')
        # "
    )

    return LaunchDescription(
        [
            pixhawk_address_arg,
            gcs_url_arg,
            zed_odom_topic_arg,
            LogInfo(
                msg=["[zed_mavros_vio] FCU → serial://", pixhawk_address, ":57600"]
            ),
            LogInfo(msg=["[zed_mavros_vio] ZED odom → ", zed_odom_topic]),
            LogInfo(
                msg=(
                    "[zed_mavros_vio] Arm in MANUAL/STABILIZE first, "
                    "then switch to GUIDED (ArduSub pre-4.5.7 will dive if armed in GUIDED)"
                )
            ),
            static_tf_node,
            mavros_node,
        ]
    )
