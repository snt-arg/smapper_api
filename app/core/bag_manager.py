import datetime
import os
import base62
import uuid
import yaml
from typing import Dict, List, Optional

from app.core.services.rosbag_service import RosbagService
from app.core.services.service import Service, ServiceState
from app.schemas.bags import (
    BagCreationResponse,
    BagSchema,
    MinimalRosbagMetadata,
    BagRecordingRequestSchema,
    RosbagMetadata,
)
from app.schemas.ros import TopicSchema
from app.exceptions import (
    BagNotFoundException,
    BagRecordingOnGoingException,
)
from app.logger import logger


class BagManager:
    """Manages ROS2 bag recordings: creating, storing, reading, and stopping recordings."""

    bags: Dict[str, BagSchema] = {}
    storage_path: str
    rosbag_service: Optional[Service] = None
    recording_bag_metadata: Optional[RosbagMetadata] = None

    def __init__(self, storage_path: str):
        """Initialize the bag manager and ensure the storage directory exists.

        Args:
            storage_path: Path to the directory where rosbag data will be stored.
        """
        storage_path = os.path.expandvars(storage_path)
        if not os.path.exists(storage_path):
            self._create_storage_dir(storage_path)
        assert os.path.exists(storage_path), "Storage path does not exist"
        self.storage_path = storage_path

        self.__read_bags_storage()

    def _create_storage_dir(self, path: str) -> None:
        """Create the storage directory if it doesn't exist.

        Args:
            path: Path to create.
        """
        os.makedirs(path, exist_ok=True)

    def get_recording_state(self) -> ServiceState:
        """Return the current state of the rosbag recording service."""
        if self.rosbag_service is None:
            return ServiceState.INACTIVE
        return self.rosbag_service.get_state()

    def create_bag(self, request: BagRecordingRequestSchema) -> BagCreationResponse:
        """Start recording a new rosbag.

        Args:
            request: Bag creation request schema including topics and metadata.

        Returns:
            A response containing the bag ID and service state.

        Raises:
            BagRecordingOnGoingException: If a recording is already in progress.
            Exception: If the rosbag fails to start.
        """
        if self.rosbag_service and self.rosbag_service.is_running():
            raise BagRecordingOnGoingException()

        bag_id = self.__generate_bag_id()
        bag_name = self.__generate_bag_name(bag_id, request.name)

        self.rosbag_service = RosbagService(self.storage_path, bag_name, request.topics)
        self.rosbag_service.start()

        self.rosbag_service.poll()
        if not self.rosbag_service.is_running():
            raise Exception("Failed to start rosbag record")

        self.recording_bag_metadata = RosbagMetadata(id=bag_id, **request.model_dump())

        return BagCreationResponse(
            bag_id=bag_id, service_state=self.rosbag_service.get_state().name
        )

    def stop_bag_recording(self) -> BagSchema:
        """Stop the active rosbag recording.

        Returns:
            The BagSchema of the stopped bag.

        Raises:
            Exception: If there is no active recording to stop.
        """
        if not self.rosbag_service or not self.rosbag_service.is_running():
            raise Exception("No active recording to stop")
        self.rosbag_service.stop()

        # FIX: Don't use assert but throw an Exception
        assert self.recording_bag_metadata is not None

        self.__read_bags_storage()
        return self.get_bag_by_id(self.recording_bag_metadata.id)

    def get_bags(self) -> List[BagSchema]:
        """Return a list of all stored bag schemas."""
        return [bag for _, bag in self.bags.items()]

    def get_bag_by_id(self, id: str) -> BagSchema:
        """Retrieve a bag by its ID.

        Args:
            id: The unique ID of the bag.

        Returns:
            The corresponding BagSchema.

        Raises:
            BagNotFoundException: If the bag with the given ID is not found.
        """
        bag = self.bags.get(id)
        if bag is None:
            raise BagNotFoundException(id)

        return bag

    def _build_rosbag_path(self, name) -> str:
        """Construct the absolute path for a given rosbag name.

        Args:
            name: Name of the rosbag.

        Returns:
            Absolute path to the bag directory.
        """
        return os.path.join(self.storage_path, name)

    def _read_rosbag_metadata(self, path: str) -> MinimalRosbagMetadata:
        """Read metadata from a rosbag folder.

        Args:
            path: Filesystem path to the rosbag folder.

        Returns:
            Parsed minimal rosbag metadata.
        """
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
                TopicSchema(
                    name=name,
                    msg_type=msg_type,
                    message_count=message_count,
                    hz=0,
                    status="UNDEFINED",
                )
            )

        return MinimalRosbagMetadata(
            duration=metadata["duration"]["nanoseconds"],
            unix_timestamp=metadata["starting_time"]["nanoseconds_since_epoch"],
            topics=topics,
            message_count=metadata["message_count"],
            db_path=metadata["relative_file_paths"][0],
        )

    def __read_bags_storage(self) -> None:
        """Scan the storage directory and populate internal `bags` dictionary with existing bags."""

        for bag_name in os.listdir(self.storage_path):
            bag_path = self._build_rosbag_path(bag_name)
            rosbag_metadata = self._read_rosbag_metadata(bag_path)

            id = self.__extract_id_from_name(bag_name)

            if id is None:
                logger.warning("Failed to find ID from bag name. Creating one.")
                id = self.__generate_bag_id()
                bag_name = self.__generate_bag_name(id, bag_name, False)
                bag_path = self._build_rosbag_path(bag_name)
                new_bag_dir = os.path.join(self.storage_path, bag_name)
                os.rename(bag_path, new_bag_dir)

            bag_size_gb = (
                os.stat(os.path.join(bag_path, rosbag_metadata.db_path)).st_size / 1e9
            )
            bag_size = f"{bag_size_gb:.2f} GB"

            bag = BagSchema(
                id=id,
                name="_".join(bag_name.split("_")[1:]),
                bag_size=bag_size,
                rosbag_metadata=rosbag_metadata,
            )

            self.bags[id] = bag

    def __extract_id_from_name(self, name: str) -> Optional[str]:
        """Extract the unique bag ID from the bag folder name.

        Args:
            name: Bag folder name.

        Returns:
            A valid base62-encoded ID or None if invalid.
        """
        splits = name.split("_")
        if len(splits) == 0:
            return
        id = splits[0]

        try:
            base62.decode(id)
        except ValueError:
            return

        return id

    def __generate_bag_id(self) -> str:
        """Generate a unique base62-encoded bag ID.

        Returns:
            A new bag ID.
        """
        return base62.encode(uuid.uuid4().int >> 64)

    def __generate_bag_name(self, id: str, name: str, include_date: bool = True) -> str:
        """Construct a formatted bag name using ID, user-provided name, and date.

        Args:
            id: Unique bag ID.
            name: User-defined base name for the bag.
            include_date: Whether to include the current date.

        Returns:
            The full formatted name of the bag.
        """
        date = datetime.datetime.now().strftime("%Y.%m.%d")
        return id + "_" + name + ("_" + date if include_date else "")
