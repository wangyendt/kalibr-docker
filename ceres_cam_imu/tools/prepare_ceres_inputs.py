#!/usr/bin/env python3
"""Prepare ceres_cam_imu CSV inputs from pkl, bag, or EuRoC-style data.

The C++ calibration binary intentionally consumes neutral CSV files.  This
wrapper runs the Kalibr Docker image only for decoding Kalibr pickle objects,
ROS bags, or EuRoC image folders into those CSV files.
"""

import argparse
import pathlib
import shlex
import subprocess
import sys


KALIBR_IMAGE = "kalibr-camera-calibration:20.04"


def repo_dir():
    return pathlib.Path(__file__).resolve().parents[2]


def quote_command(command):
    return " ".join(shlex.quote(str(part)) for part in command)


def run(command, print_only=False):
    print(quote_command(command), flush=True)
    if print_only:
        return 0
    return subprocess.call(command)


def add_path_mounts(command, paths):
    """Append read-only parent mounts and return container paths."""
    mapped = []
    parent_to_mount = {}
    for path in paths:
        resolved = pathlib.Path(path).expanduser().resolve()
        parent = resolved.parent
        if parent not in parent_to_mount:
            mount_name = f"/mnt/input{len(parent_to_mount)}"
            parent_to_mount[parent] = mount_name
            command.extend(["-v", f"{parent}:{mount_name}:ro"])
        mapped.append(parent_to_mount[parent] + "/" + resolved.name)
    return mapped


def base_docker_command(args, out_dir):
    command = [
        "docker",
        "run",
        "--rm",
        "--platform",
        args.platform,
        "-v",
        f"{repo_dir()}:/repo:ro",
        "-v",
        f"{out_dir}:/out",
        "-w",
        "/out",
    ]
    return command


def prepare_pkl(args):
    if not args.corner_pkl:
        raise ValueError("--corner-pkl is required for --source-type pkl")
    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    for index, corner_pkl in enumerate(args.corner_pkl):
        command = base_docker_command(args, out_dir)
        (container_pkl,) = add_path_mounts(command, [corner_pkl])
        command.extend([args.image, "/bin/bash", "-lc"])
        corners_name = f"cam{index}_corners.csv"
        script = [
            "python3",
            "/repo/ceres_cam_imu/tools/export_kalibr_corners.py",
            "--corner-pkl",
            container_pkl,
            "--corners-csv",
            f"/out/{corners_name}",
        ]
        if index == 0:
            script.extend(["--poses-csv", "/out/cam0_corner_poses.csv"])
        command.append(quote_command(script))
        rc = run(command, args.print_only)
        if rc != 0:
            return rc
    return 0


def prepare_bag(args):
    for name, value in [
        ("--bag", args.bag),
        ("--cams", args.cams),
        ("--imu", args.imu),
        ("--target", args.target),
    ]:
        if not value:
            raise ValueError(f"{name} is required for --source-type bag")
    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    command = base_docker_command(args, out_dir)
    container_bag, container_cams, container_imu, container_target = add_path_mounts(
        command, [args.bag, args.cams, args.imu, args.target]
    )
    command.extend([args.image, "/bin/bash", "-lc"])
    script = [
        "python3",
        "/repo/ceres_cam_imu/tools/export_kalibr_bag_to_ceres.py",
        "--bag",
        container_bag,
        "--cams",
        container_cams,
        "--imu",
        container_imu,
        "--target",
        container_target,
        "--out-dir",
        "/out",
    ]
    if args.bag_from_to:
        script.extend(["--bag-from-to", str(args.bag_from_to[0]), str(args.bag_from_to[1])])
    if args.bag_freq is not None:
        script.extend(["--bag-freq", str(args.bag_freq)])
    if args.perform_synchronization:
        script.append("--perform-synchronization")
    command.append(quote_command(script))
    return run(command, args.print_only)


def prepare_euroc(args):
    for name, value in [
        ("--euroc-dir", args.euroc_dir),
        ("--cams", args.cams),
        ("--imu", args.imu),
        ("--target", args.target),
    ]:
        if not value:
            raise ValueError(f"{name} is required for --source-type euroc")
    euroc_dir = pathlib.Path(args.euroc_dir).expanduser().resolve()
    mav0 = euroc_dir / "mav0"
    if not mav0.is_dir():
        mav0 = euroc_dir
    if not (mav0 / "cam0" / "data").is_dir():
        raise ValueError(f"EuRoC cam0 data directory not found under {mav0}")
    if not (mav0 / "imu0" / "data.csv").is_file():
        raise ValueError(f"EuRoC imu0/data.csv not found under {mav0}")

    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    command = base_docker_command(args, out_dir)
    container_mav0, container_cams, container_imu, container_target = add_path_mounts(
        command, [mav0, args.cams, args.imu, args.target]
    )
    command.extend([args.image, "/bin/bash", "-lc"])
    bag_name = pathlib.PurePosixPath(args.output_bag or "euroc_input.bag").name
    cam1_stage = ""
    if (mav0 / "cam1" / "data").is_dir():
        cam1_stage = "mkdir -p /out/euroc_stage/cam1; ln -sf {}/cam1/data/* /out/euroc_stage/cam1; ".format(
            shlex.quote(container_mav0)
        )
    export_args = [
        "python3",
        "/repo/ceres_cam_imu/tools/export_kalibr_bag_to_ceres.py",
        "--bag",
        f"/out/{bag_name}",
        "--cams",
        container_cams,
        "--imu",
        container_imu,
        "--target",
        container_target,
        "--out-dir",
        "/out",
    ]
    if args.bag_from_to:
        export_args.extend(["--bag-from-to", str(args.bag_from_to[0]), str(args.bag_from_to[1])])
    if args.bag_freq is not None:
        export_args.extend(["--bag-freq", str(args.bag_freq)])
    if args.perform_synchronization:
        export_args.append("--perform-synchronization")
    script = (
        "set -e; "
        "rm -rf /out/euroc_stage; "
        "mkdir -p /out/euroc_stage/cam0; "
        f"ln -sf {shlex.quote(container_mav0)}/cam0/data/* /out/euroc_stage/cam0; "
        + cam1_stage
        + f"cp {shlex.quote(container_mav0)}/imu0/data.csv /out/euroc_stage/imu0.csv; "
        + f"/catkin_ws/devel/lib/kalibr/kalibr_bagcreater --folder /out/euroc_stage --output-bag /out/{shlex.quote(bag_name)}; "
        + quote_command(export_args)
    )
    command.append(script)
    return run(command, args.print_only)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert pkl, bag, or EuRoC-style datasets to Ceres CSV inputs."
    )
    parser.add_argument("--source-type", choices=["pkl", "bag", "euroc"], required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--image", default=KALIBR_IMAGE)
    parser.add_argument("--platform", default="linux/amd64")
    parser.add_argument("--print-only", action="store_true")
    parser.add_argument("--corner-pkl", action="append", default=[])
    parser.add_argument("--bag")
    parser.add_argument("--euroc-dir")
    parser.add_argument("--output-bag")
    parser.add_argument("--cams")
    parser.add_argument("--imu")
    parser.add_argument("--target")
    parser.add_argument("--bag-from-to", type=float, nargs=2)
    parser.add_argument("--bag-freq", type=float)
    parser.add_argument("--perform-synchronization", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        if args.source_type == "pkl":
            return prepare_pkl(args)
        if args.source_type == "bag":
            return prepare_bag(args)
        if args.source_type == "euroc":
            return prepare_euroc(args)
        raise ValueError(f"unsupported source type: {args.source_type}")
    except Exception as exc:
        print(f"prepare_ceres_inputs failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
