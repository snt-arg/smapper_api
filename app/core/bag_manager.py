import os
import yaml
from typing import Dict, List

from app.schemas.bags import BagSchema, MinimalRosbagMetadata
from app.schemas.ros import TopicSchema
from app.exceptions import BagNotFoundException, NotYetImplementedException
from app.logger import logger


class BagManager:
    bags: Dict[str, BagSchema] = {}
    storage_path: str

    def __init__(self, storage_path: str):
        assert os.path.exists(storage_path), "Storage path does not exist"
        self.storage_path = storage_path

        self._read_bags_storage()

    def get_bags(self) -> List[BagSchema]:
        return [bag for _, bag in self.bags.items()]

    def get_bag_by_id(self, id: str) -> BagSchema:
        bag = self.bags.get(id)
        if bag is None:
            raise BagNotFoundException(id)

        return bag

    def _build_rosbag_path(self, name) -> str:
        return os.path.join(self.storage_path, name)

    def _read_rosbag_metadata(self, path: str) -> MinimalRosbagMetadata:
        file = os.path.join(path, "metadata.yaml")
        with open(file, "r") as f:
            metadata = yaml.safe_load(f)

        metadata = metadata["rosbag2_bagfile_information"]

        topics: List[TopicSchema] = []
        for topic in metadata["topics_with_message_count"]:
            topic_metadata = topic["topic_metadata"]
            name = topic_metadata["name"]
            msg_type = topic_metadata["type"]
            message_count = topic["message_count"]
            topics.append(
                TopicSchema(name=name, msg_type=msg_type, message_count=message_count)
            )

        return MinimalRosbagMetadata(
            duration=metadata["duration"]["nanoseconds"],
            unix_timestamp=metadata["starting_time"]["nanoseconds_since_epoch"],
            topics=topics,
            message_count=metadata["message_count"],
            db_path=os.path.join(path, metadata["relative_file_paths"][0]),
        )

    def _read_bags_storage(self) -> None:
        for bag_name in os.listdir(self.storage_path):
            rosbag_metadata = self._read_rosbag_metadata(
                self._build_rosbag_path(bag_name)
            )

            # TODO: Use the correct id when available
            id = bag_name

            bag_size_gb = os.stat(rosbag_metadata.db_path).st_size / 1e9
            bag_size = f"{bag_size_gb:.2f} GB"

            bag = BagSchema(
                id=id,
                name=bag_name,
                bag_size=bag_size,
                rosbag_metadata=rosbag_metadata,
            )

            self.bags[id] = bag
