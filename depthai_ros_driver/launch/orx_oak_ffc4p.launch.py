"""Launch one or more OAK-FFC-4P devices, optionally hardware-synced via FSIN.

  trigger  i_fsync_trigger (snapshot, one exposure per FSIN edge),
           i_fps = sync.trigger_fps (sensor frame period, not the frame rate)
  fsync    i_fsync_continuous + i_fsync_mode INPUT, i_fps = sync.pulse_rate_hz
  none     both off, i_fps from the sensor section

Examples:
  ros2 launch depthai_ros_driver orx_oak_ffc4p.launch.py
  ros2 launch depthai_ros_driver orx_oak_ffc4p.launch.py sync_mode:=none
  ros2 launch depthai_ros_driver orx_oak_ffc4p.launch.py params_file:=/path/ffc.yaml
"""

import copy
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

SYNC_MODES = ("none", "trigger", "fsync")
ALL_SENSORS = ("rgb", "left", "right", "cam_d")
CAM_ARRAY = "depthai_ros_driver::pipeline_gen::CamArray"


def sensor_modes(sync, override):
    """Return {sensor: mode} from the sync block and the sync_mode override."""
    default = sync.get("mode", "none")
    sensors = sync.get("sensors", list(ALL_SENSORS))
    if isinstance(sensors, dict):
        modes = {s: (m or default) for s, m in sensors.items()}
    else:
        modes = {s: default for s in sensors}
    if override:
        modes = {s: override for s in modes}
    for s, m in modes.items():
        if m not in SYNC_MODES:
            raise RuntimeError(f"sync mode for '{s}' must be one of {SYNC_MODES}, got '{m}'")
    return modes


def apply_sync(params, sync, modes):
    """Write the trigger parameters into the per-sensor parameter sections."""
    for sensor, mode in modes.items():
        p = params.setdefault(sensor, {})
        p["i_fsync_trigger"] = mode == "trigger"
        p["i_fsync_continuous"] = mode == "fsync"
        if mode == "none":
            continue
        if mode == "trigger":
            p["i_fps"] = float(sync.get("trigger_fps", 60.0))
            p["i_num_frames_burst"] = 1
            p["i_num_frames_discard"] = 0
        else:  # fsync
            p["i_fps"] = float(sync.get("pulse_rate_hz", 20.0))
            p["i_fsync_mode"] = "INPUT"
        # Start of exposure per frame, for the IMU-camera time alignment.
        p["i_publish_exposure_start"] = True
        p.setdefault("i_add_exposure_offset", True)
        # Publish every frame (pairing needs them all), stamped on the host clock.
        p["i_enable_lazy_publisher"] = False
        p["i_get_base_device_timestamp"] = False


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    override = LaunchConfiguration("sync_mode").perform(context).strip()
    namespace = LaunchConfiguration("namespace").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"
    if override and override not in SYNC_MODES:
        raise RuntimeError(f"sync_mode must be empty or one of {SYNC_MODES}, got '{override}'")

    with open(params_file, "r") as f:
        cfg = yaml.safe_load(f) or {}

    actions = []
    for key, entry in cfg.items():
        name = "oak" if "*" in str(key) else str(key).strip("/")
        entry = entry or {}
        params = copy.deepcopy(entry.get("ros__parameters", {}))
        sync = entry.get("sync", {}) or {}

        camera = params.setdefault("camera", {})
        camera.setdefault("i_pipeline_type", CAM_ARRAY)
        if camera["i_pipeline_type"] == "CamArray":
            camera["i_pipeline_type"] = CAM_ARRAY  # bypass the 1/2-sensor validator

        modes = sensor_modes(sync, override)
        apply_sync(params, sync, modes)
        summary = ", ".join(f"{s}={m}" for s, m in modes.items()) or "no sensors listed"
        actions.append(LogInfo(msg=f"[{name}] FFC-4P sync: {summary}"))

        actions.append(
            ComposableNodeContainer(
                name=f"{name}_container",
                namespace=namespace,
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name,
                        namespace=namespace,
                        parameters=[params],
                    )
                ],
                arguments=["--ros-args", "--log-level", log_level],
                output="both",
            )
        )
    if not actions:
        raise RuntimeError(f"No device entries in {params_file}")
    return actions


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("depthai_ros_driver"), "config", "oak_ffc4p.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Device config, one top-level entry per FFC-4P (see config/oak_ffc4p.yaml).",
            ),
            DeclareLaunchArgument(
                "sync_mode",
                default_value="",
                description="Override sync.mode for all listed sensors of all devices: "
                "none | trigger | fsync. Empty = use the params file.",
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=launch_setup),
        ]
    )
