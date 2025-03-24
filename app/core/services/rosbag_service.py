from typing import List, Optional
import os

from .service import Service


class RosbagService(Service):
    def __init__(
        self, output_dir: str, name: str, topics_to_record: Optional[List[str]] = None
    ) -> None:
        self.topics = " ".join(topics_to_record) if topics_to_record else "-a"
        self.output = os.path.join(output_dir, name)
        cmd = f"ros2 bag record -o {self.output} {self.topics}"
        super().__init__(
            name="Rosbag Service",
            id="rosbag_service",
            srv_type="ROSBAG_SERVICE",
            cmd=cmd,
        )
