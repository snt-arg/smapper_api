# SMapper API

The **SMapper API** is a FastAPI-based application that offers a RESTful interface for controlling the SMapper device (or any similar devices).
The primary goal of this API is to provide a straightforward interface to start and stop various services, such as ROS2 nodes or simple shell commands.

Key features include:

- Start/Stop ROS2 Node Services: Control ROS2 nodes and related services easily.
- ROS Bag Recordings: Start and stop ROS bag recordings, while automatically tracking metadata in an SQLite database.
- ROS Topic Monitoring: Monitor available ROS topics and retrieve metadata such as status, type, and frequency (Hz).

This API is designed to be extensible, allowing integration with other devices and systems, while simplifying interactions with ROS2 and related services.

## Installation

This project uses the UV package manager to manage dependencies. You can install UV [here](https://docs.astral.sh/uv/getting-started/installation/).

## Usage

> [!NOTE]
> The following commands are to be executed from the root directory of the project.

For development purposes use

```sh
uv run fastapi dev
```

For production purposes use

```sh
uv fastapi run
```
