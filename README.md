# SMapper API

The **SMapper API** is a FastAPI-based application that offers a RESTful interface for controlling the SMapper device or other similar systems. The primary goal of this API is to provide a straightforward, web-based interface to manage services, such as ROS2 nodes or simple shell commands.

### Key Features

- **Service Management:** Easily start and stop background services (e.g., ROS2 nodes) through simple API endpoints.
- **ROS Bag Recording:** Manage ROS bag recordings and automatically track metadata (like topics, duration, and file size) in a local SQLite database.
- **ROS Topic Monitoring:** Monitor available ROS topics and retrieve metadata such as their status, message type, and publishing frequency (Hz).
- **Extensible Configuration:** Define custom services, commands, and topic presets through simple YAML configuration files.

## Prerequisites & Installation

Before you begin, ensure your system meets the following requirements.

### 1. ROS2 Humble

The API is built to interface directly with ROS2 services. You must have **ROS2 Humble Hawksbill** installed on your system.

- You can find the official installation instructions at [docs.ros.org/en/humble/Installation](https://docs.ros.org/en/humble/Installation.html).

### 2. UV Package Manager

This project uses **UV** to manage its Python dependencies. UV is a fast package manager from Astral.

1.  **Install UV:** If you don't have it, you can install UV by following the instructions [here](https://docs.astral.sh/uv/getting-started/installation/).
2.  **Install Dependencies:** Navigate to the project's root directory and run any `uv sync` command. UV will automatically create a virtual environment and install all required packages listed in `pyproject.toml`.

## Configuration

The API's behavior is controlled by YAML configuration files located in the `config/` directory.

- **Default Configuration:** By default, the API loads its settings from `config/settings.yaml`. This file is where you can define available services, the commands they execute, and presets for ROS bag recordings.

- **Custom Configurations:** You can create multiple configuration files (e.g., `config/another_settings.yaml`). To instruct the API to use a different file, set the `API_SETTINGS_NAME` environment variable to the name of your file (without the `.yaml` extension).

    For example:

    ```sh
    export API_SETTINGS_NAME=another_settingsl.yaml
    # Now when the API starts, it will load config/another_settings.yaml
    ```

## Usage

> [!NOTE]
> All commands should be executed from the root directory of the project.

The API can be run in two modes: development and production.

### Development Mode

For development, use the `dev` command. This will start the server with hot-reloading enabled, which automatically restarts the server whenever you make changes to the code.

```sh
uv run fastapi dev
```

### Production Mode

For production environments, use the run command. This starts the API using the unicorn server, which is optimized for stability and performance.

```bash
uv run fastapi run
```

## Deployment

For deploying the API as a service that starts automatically on system boot, a helper script is provided.

The `deploy.sh` script, located in the project's root directory, is designed to be used by a systemd service or a similar init system. It sets up the necessary environment and launches the API in production mode, ensuring it is running after the device boots up.
