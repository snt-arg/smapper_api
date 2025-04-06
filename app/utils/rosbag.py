import os
from typing import List
import yaml
import datetime

from app.schemas.bags import MinimalRosbagMetadata
from app.schemas.ros import Topic


def get_rosbag_db_path(path: str) -> str:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Rosbag path {path} does not exists.")

    files = os.listdir(path)

    for file in files:
        if file.endswith(".db3"):
            return os.path.join(path, file)

    raise Exception("Could not find a .db file")


def get_rosbag_metadata_path(path: str) -> str:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Rosbag path {path} does not exists.")

    files = os.listdir(path)

    for file in files:
        if file.endswith((".yml", ".yaml")):
            return os.path.join(path, file)

    raise Exception("Could not find a .db file")


def get_rosbag_size(path: str) -> int:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Rosbag path {path} does not exists.")

    db_file = get_rosbag_db_path(path)

    size_bytes = os.stat(db_file).st_size

    return size_bytes


def create_rosbag_name_with_date(name: str, epoch_time: float) -> str:
    dt_object = datetime.datetime.fromtimestamp(epoch_time)
    formatted_time = dt_object.strftime("%Y-%m-%d_%H-%M-%S")
    return name + "_" + formatted_time


def read_rosbag_metadata(path: str) -> MinimalRosbagMetadata:
    """Read metadata from a rosbag folder.

    Args:
        path: Filesystem path to the rosbag folder.

    Returns:
        Parsed minimal rosbag metadata.
    """
    file = get_rosbag_metadata_path(path)
    with open(file, "r") as f:
        metadata = yaml.safe_load(f)

    metadata = metadata["rosbag2_bagfile_information"]

    topics: List[Topic] = []
    for topic in metadata["topics_with_message_count"]:
        topic_metadata = topic["topic_metadata"]
        name = topic_metadata["name"]
        msg_type = topic_metadata["type"]
        message_count = topic["message_count"]
        topics.append(
            Topic(
                name=name,
                msg_type=msg_type,
            )
        )

    return MinimalRosbagMetadata(
        duration=metadata["duration"]["nanoseconds"],
        start_time=metadata["starting_time"]["nanoseconds_since_epoch"],
        topics=topics,
        message_count=metadata["message_count"],
        db_path=metadata["relative_file_paths"][0],
    )
