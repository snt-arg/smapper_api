# app/core/processes/logger.py
import logging
import colorlog


def get_logger(process_name: str) -> logging.Logger:
    """
    Create and configure a custom logger for a process with colored output.

    Args:
        process_name (str): The name of the process.

    Returns:
        logging.Logger: A configured logger instance.
    """
    # Create a logger
    logger = logging.getLogger(f"process_{process_name}")
    logger.setLevel(logging.INFO)  # Set the logging level

    # Define the log format with colors
    log_format = (
        "%(log_color)s%(levelname)s%(reset)s "
        "[process:%(process_name)s]: "
        "%(message)s"
    )

    # Create a colored formatter
    formatter = colorlog.ColoredFormatter(
        log_format,
        datefmt="%Y-%m-%d %H:%M:%S",
        log_colors={
            "DEBUG": "cyan",
            "INFO": "green",
            "WARNING": "yellow",
            "ERROR": "red",
            "CRITICAL": "red,bg_white",
        },
    )

    # Create a console handler and set the formatter
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)

    # Add the handler to the logger
    logger.addHandler(console_handler)

    # Add the process_name to the logger's extra context
    logger = logging.LoggerAdapter(logger, {"process_name": process_name})

    return logger
