import os
from typing import List, Optional, Dict

from app.logging import logger
from app.schemas.recording import BagRecorderCompressionSettings

from .ros_service import RosService


class RosbagService(RosService):
    """A service that records ROS2 topics using `ros2 bag`.

    This service wraps the ROS2 bag recording functionality and can either record
    all topics or a specific list of topics to a given output directory.
    """

    def __init__(
        self,
        output_dir: str,
        name: str,
        topics_to_record: List[str],
        compression: Optional[BagRecorderCompressionSettings],
        ws: Optional[str],
        env: Optional[Dict[str, str]] = None,
    ) -> None:
        """Initialize the RosbagService.

        Args:
            output_dir: Directory where the rosbag files will be stored.
            name: Name of the output rosbag file/directory.
            topics_to_record: Optional list of topic names to record. If None, all topics will be recorded.
        """
        self.topics = " ".join(topics_to_record) if len(topics_to_record) > 0 else "-a"
        self.output = os.path.join(output_dir, name)

        if compression and compression.enabled:
            cmd = f"record -s mcap --compression-mode {compression.mode} --compression-format {compression.format} -o {self.output} {self.topics}"
        else:
            cmd = f"record -s mcap --storage-preset-profile zstd_fast -o {self.output} {self.topics}"
        logger.info(f"Rosbag command: {cmd}")
        super().__init__(
            id="rosbag_service",
            name="Rosbag Service",
            auto_start=False,
            restart_on_failure=False,
            exec_type="BAG",
            exec=cmd,
            ws=ws,
            env=env,
        )
