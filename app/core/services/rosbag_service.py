from typing import List, Optional
import os

from .service import Service


class RosbagService(Service):
    """A service that records ROS2 topics using `ros2 bag`.

    This service wraps the ROS2 bag recording functionality and can either record
    all topics or a specific list of topics to a given output directory.
    """

    def __init__(
        self, output_dir: str, name: str, topics_to_record: Optional[List[str]] = None
    ) -> None:
        """Initialize the RosbagService.

        Args:
            output_dir: Directory where the rosbag files will be stored.
            name: Name of the output rosbag file/directory.
            topics_to_record: Optional list of topic names to record. If None, all topics will be recorded.
        """
        self.topics = " ".join(topics_to_record) if topics_to_record else "-a"
        self.output = os.path.join(output_dir, name)
        cmd = f"ros2 bag record -o {self.output} {self.topics}"
        super().__init__(
            name="Rosbag Service",
            id="rosbag_service",
            cmd=cmd,
        )
