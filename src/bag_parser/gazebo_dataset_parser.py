#!/usr/bin/env python3
"""Convert recorded Gazebo rosbag files into tabular training datasets."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence


CMD_TOPIC = "/cmd_vel"
TERRAIN_TOPIC = "/terrain/surface_type"
ODOM_TOPICS = (
    "/odometry/filtered",
    "/ground_truth/odom",
    "/warthog_velocity_controller/odom",
)
DATASET_COLUMNS = (
    "stamp",
    "terrain",
    "cmd_vx",
    "cmd_wz",
    "odom_vx",
    "odom_wz",
    "time_sec",
    "dt",
)


@dataclass(frozen=True)
class ProjectPaths:
    workspace_dir: Path
    bags_dir: Path
    datasets_dir: Path


def project_paths() -> ProjectPaths:
    workspace_dir = find_workspace_dir(Path(__file__).resolve())
    return ProjectPaths(
        workspace_dir=workspace_dir,
        bags_dir=workspace_dir / "bags",
        datasets_dir=workspace_dir / "datasets",
    )


def find_workspace_dir(start: Path) -> Path:
    for candidate in [start.parent, *start.parents]:
        if (candidate / "src" / "package.xml").exists():
            return candidate
        if candidate.name == "src" and (candidate / "package.xml").exists():
            return candidate.parent

    cwd = Path.cwd().resolve()
    for candidate in [cwd, *cwd.parents]:
        if (candidate / "src" / "package.xml").exists():
            return candidate
    raise RuntimeError("Could not find workspace root with src/package.xml")


def stamp_to_ns(stamp) -> int:
    sec = int(getattr(stamp, "sec", 0))
    nsec = int(getattr(stamp, "nanosec", getattr(stamp, "nsec", 0)))
    return sec * 1_000_000_000 + nsec


def odom_velocity_row(msg, stamp: int) -> Dict[str, object]:
    return {
        "stamp": stamp,
        "odom_vx": msg.twist.twist.linear.x,
        "odom_wz": msg.twist.twist.angular.z,
    }


def select_odom_topic(available_topics: Iterable[str]) -> str:
    available = set(available_topics)
    for topic in ODOM_TOPICS:
        if topic in available:
            return topic
    raise RuntimeError(
        "No supported odometry topic found in bag. Expected one of: "
        + ", ".join(ODOM_TOPICS)
    )


def process_gazebo_bag_file(bag_file_source: Path):
    from rosbags.highlevel import AnyReader

    cmd_rows: List[Dict[str, object]] = []
    odom_rows: List[Dict[str, object]] = []
    terrain_rows: List[Dict[str, object]] = []

    with AnyReader([bag_file_source]) as reader:
        available_topics = {connection.topic for connection in reader.connections}
        odom_topic = select_odom_topic(available_topics)
        expected_topics = {CMD_TOPIC, TERRAIN_TOPIC, odom_topic}
        missing_topics = expected_topics - available_topics
        if missing_topics:
            raise RuntimeError(
                "Bag is missing required topics: " + ", ".join(sorted(missing_topics))
            )

        connections = [
            connection
            for connection in reader.connections
            if connection.topic in expected_topics
        ]

        for connection, bag_time, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)

            if connection.topic == CMD_TOPIC:
                cmd_rows.append(
                    {
                        "stamp": bag_time,
                        "cmd_vx": msg.linear.x,
                        "cmd_wz": msg.angular.z,
                    }
                )
            elif connection.topic == odom_topic:
                stamp = stamp_to_ns(msg.header.stamp) if msg.header.stamp else bag_time
                odom_rows.append(odom_velocity_row(msg, stamp))
            elif connection.topic == TERRAIN_TOPIC:
                terrain_rows.append(
                    {
                        "stamp": bag_time,
                        "terrain": msg.data,
                    }
                )

    df_cmd = dataframe(cmd_rows, "cmd_vel")
    df_odom = dataframe(odom_rows, "odometry")
    df_terrain = dataframe(terrain_rows, "terrain")
    return df_cmd, df_odom, df_terrain


def dataframe(rows: Sequence[Dict[str, object]], label: str):
    import pandas as pd

    if not rows:
        raise RuntimeError(f"No {label} messages found in bag")
    return pd.DataFrame(rows).sort_values("stamp").reset_index(drop=True)


def process_and_merge_gazebo_dataset(
    gazebo_bag_src: Path,
    csv_filename: Path,
):
    import pandas as pd

    df_cmd, df_odom, df_terrain = process_gazebo_bag_file(gazebo_bag_src)

    df = pd.merge_asof(
        df_terrain,
        df_cmd,
        on="stamp",
        direction="nearest",
    )
    df = pd.merge_asof(
        df,
        df_odom,
        on="stamp",
        direction="nearest",
    )

    df["time_sec"] = (df["stamp"] - df["stamp"].iloc[0]) / 1e9
    df["dt"] = df["stamp"].diff().shift(-1) / 1e9
    df = df.dropna().reset_index(drop=True)
    df = df.loc[:, DATASET_COLUMNS]

    csv_filename.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(csv_filename, index=False)
    return df


def target_name(name: str) -> str:
    path = Path(name)
    if path.name != name or name in {".", ".."}:
        raise ValueError("Use a world or world-set name only, without directories")
    if not name:
        raise ValueError("World or world-set name cannot be empty")
    return name


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Parse ./bags/<name>.bag or all bags from ./bags/<name>/ "
            "into the matching path under ./datasets/."
        )
    )
    parser.add_argument(
        "name",
        help="World or world-set name",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    paths = project_paths()
    name = target_name(args.name)
    single_bag = paths.bags_dir / f"{name}.bag"
    bag_dir = paths.bags_dir / name

    if single_bag.is_file():
        jobs = [(single_bag, paths.datasets_dir / f"{name}.csv")]
    elif bag_dir.is_dir():
        bag_paths = sorted(bag_dir.glob("*.bag"))
        if not bag_paths:
            raise FileNotFoundError(f"No .bag files found in {bag_dir}")
        dataset_dir = paths.datasets_dir / name
        jobs = [
            (bag_path, dataset_dir / f"{bag_path.stem}.csv")
            for bag_path in bag_paths
        ]
    else:
        raise FileNotFoundError(
            f"Input bag was not found: {single_bag}; "
            f"input bag directory was not found: {bag_dir}"
        )

    for bag_path, dataset_path in jobs:
        df = process_and_merge_gazebo_dataset(bag_path, dataset_path)
        print(f"Wrote {len(df)} rows to {dataset_path}")


if __name__ == "__main__":
    main()
