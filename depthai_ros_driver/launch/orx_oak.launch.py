"""Launch any number of OAK devices (OAK-D Pro W, OAK-FFC-4P, ...) from one config.

Without a `sync:` block no sync parameter is written, the driver defaults apply.

Sync modes (FFC, hardware FSIN on the PSRB connector):
  trigger  i_fsync_trigger (snapshot, one exposure per FSIN edge, global shutter
           only), i_fps = sync.trigger_fps (sensor frame period, not the rate)
  fsync    i_fsync_continuous + i_fsync_mode INPUT, i_fps = sync.pulse_rate_hz
  none     both off, i_fps from the sensor section

Examples:
  ros2 launch depthai_ros_driver orx_oak.launch.py params_file:=/path/setup.yaml
"""

import copy
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

SYNC_MODES = ("none", "trigger", "fsync")
FFC_SENSORS = ("rgb", "left", "right", "cam_d")
CAM_ARRAY = "depthai_ros_driver::pipeline_gen::CamArray"
LAUNCH_KEYS = ("device", "sync", "ros__parameters")


def is_ffc(model):
    return model.upper().startswith("OAK-FFC")


def device_entries(cfg, params_file):
    """Return [(key, entry)] for all device entries of the params file."""
    if "ros_params" in cfg:
        cfg = cfg["ros_params"] or {}
    entries = [
        (k, v or {}) for k, v in cfg.items()
        if isinstance(v, dict) and ("ros__parameters" in v or "device" in v)
    ]
    if not entries:
        raise RuntimeError(f"No device entries (with ros__parameters) in {params_file}")
    return entries


def split_key(key, base_ns):
    """'/ns/oak1' -> ('oak1', '<base_ns>/ns'); '/**' -> ('oak', base_ns)."""
    key = str(key)
    if "*" in key:
        return "oak", base_ns
    parts = [p for p in key.strip("/").split("/") if p]
    ns = "/".join(p for p in [base_ns.strip("/")] + parts[:-1] if p)
    return parts[-1], ("/" + ns if ns else "")


def sensor_modes(sync, override, params):
    """Return {sensor: mode} from the sync block and the sync_mode override."""
    default = sync.get("mode", "none")
    sensors = sync.get("sensors", [s for s in FFC_SENSORS if s in params])
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


TF_POSE_PARAMS = (
    "i_tf_cam_pos_x", "i_tf_cam_pos_y", "i_tf_cam_pos_z",
    "i_tf_cam_roll", "i_tf_cam_pitch", "i_tf_cam_yaw",
)


def tf_params(name, device, camera):
    pose = [float(v) for v in device.get("pose", [0.0] * 6)]
    if len(pose) != 6:
        raise RuntimeError(f"[{name}] device.pose must be [x, y, z, roll, pitch, yaw]")
    tf = {
        "i_publish_tf_from_calibration": True,
        "i_tf_camera_name": name,
        "i_tf_camera_model": "",  # "" = read from the device calibration
        "i_tf_base_frame": name,
        "i_tf_parent_frame": str(
            device.get("parent_frame", camera.get("i_tf_parent_frame", "oak-d-base-frame"))
        ),
        "i_tf_imu_from_descr": str(device.get("imu_from_descr", False)).lower(),
    }
    tf.update({p: str(v) for p, v in zip(TF_POSE_PARAMS, pose)})  # the driver takes strings
    return tf, pose


def device_actions(key, entry, base_ns, override, default_log_level):
    name, namespace = split_key(key, base_ns)
    device = entry.get("device") or {}
    sync = entry.get("sync")
    params = copy.deepcopy(entry.get("ros__parameters") or {})
    unknown = set(entry) - set(LAUNCH_KEYS)
    if unknown:
        raise RuntimeError(f"[{name}] unknown keys {sorted(unknown)}, expected {LAUNCH_KEYS}")
    model = str(device.get("model", ""))
    if not model:
        raise RuntimeError(f"[{name}] device.model is required (e.g. OAK-D-PRO-W, OAK-FFC-4P)")
    ffc = is_ffc(model)
    camera = params.setdefault("camera", {})
    actions, info = [], [model]

    if ffc:
        # Any sensor combination on the FFC: CamArray via its full plugin name
        camera.setdefault("i_pipeline_type", CAM_ARRAY)
        if camera["i_pipeline_type"] == "CamArray":
            camera["i_pipeline_type"] = CAM_ARRAY

    if sync is not None:
        if not ffc:
            raise RuntimeError(f"[{name}] sync is only supported on OAK-FFC devices, not {model}")
        modes = sensor_modes(sync or {}, override, params)
        apply_sync(params, sync or {}, modes)
        info.append("sync " + (", ".join(f"{s}={m}" for s, m in modes.items()) or "no sensors"))
    elif override:
        info.append(f"no sync block, sync_mode:={override} ignored")

    publish_tf = bool(device.get("publish_tf", not ffc))
    if publish_tf:
        tf, pose = tf_params(name, device, camera)
        camera.update(tf)
        info.append(f"TF {tf['i_tf_parent_frame']} -> {name}")
    else:
        camera["i_publish_tf_from_calibration"] = False
        info.append("no TF")

    container = f"{name}_container"
    actions.append(LogInfo(msg=f"[{namespace}/{name}] " + "; ".join(info)))
    actions.append(
        ComposableNodeContainer(
            name=container,
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
            arguments=["--ros-args", "--log-level", str(device.get("log_level", default_log_level))],
            output="both",
        )
    )

    # Camera housing + base frame from depthai_descriptions (same as camera.launch.py with publish_tf_from_calibration)
    if publish_tf and not ffc:
        urdf_launch = os.path.join(
            get_package_share_directory("depthai_descriptions"), "launch", "urdf_launch.py"
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(urdf_launch),
                launch_arguments={
                    "namespace": namespace,
                    "tf_prefix": name,
                    "camera_model": model,
                    "base_frame": name,
                    "parent_frame": camera["i_tf_parent_frame"],
                    "cam_pos_x": str(pose[0]),
                    "cam_pos_y": str(pose[1]),
                    "cam_pos_z": str(pose[2]),
                    "cam_roll": str(pose[3]),
                    "cam_pitch": str(pose[4]),
                    "cam_yaw": str(pose[5]),
                    "use_composition": "true",
                    "use_base_descr": "true",
                }.items(),
            )
        )
    return name, camera.get("i_mx_id", ""), actions


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    override = LaunchConfiguration("sync_mode").perform(context).strip()
    namespace = LaunchConfiguration("namespace").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"
    if override and override not in SYNC_MODES:
        raise RuntimeError(f"sync_mode must be empty or one of {SYNC_MODES}, got '{override}'")
    if not params_file:
        raise RuntimeError("params_file is required")

    with open(params_file, "r") as f:
        cfg = yaml.safe_load(f) or {}

    actions, names, mx_ids = [], [], []
    for key, entry in device_entries(cfg, params_file):
        name, mx_id, device = device_actions(key, entry, namespace, override, log_level)
        if name in names:
            raise RuntimeError(f"Duplicate device name '{name}' in {params_file}")
        names.append(name)
        mx_ids.append(str(mx_id or ""))
        actions += device
    if len(names) > 1 and not all(mx_ids):
        raise RuntimeError(
            f"Several devices in {params_file}: each needs camera.i_mx_id, "
            "otherwise they all open the first device found"
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="Device config, one entry per device (see the module docstring).",
            ),
            DeclareLaunchArgument(
                "sync_mode",
                default_value="",
                description="Override sync.mode of all devices that have a sync block: "
                "none | trigger | fsync. Empty = use the params file.",
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=launch_setup),
        ]
    )
