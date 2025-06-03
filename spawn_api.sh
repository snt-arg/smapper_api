#!/bin/bash

source .venv/bin/activate
source /opt/ros/humble/setup.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/smapper/cyclonedds.xml
fastapi run --host 0.0.0.0 --port 8000
