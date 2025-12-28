import logging
import sys
from typing import Optional


def setup_logger(
    name: str = "rag_pipeline",
    level: int = logging.INFO,
    log_file: Optional[str] = None,
    format_string: str = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
) -> logging.Logger:
    """
    Set up a structured logger with both console and optional file output.

    Args:
        name: Name of the logger
        level: Logging level (default: INFO)
        log_file: Optional file path to write logs to
        format_string: Format string for log messages

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)

    # Avoid adding handlers multiple times
    if logger.handlers:
        return logger

    logger.setLevel(level)

    # Create formatter
    formatter = logging.Formatter(format_string)

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)

    # Add console handler to logger
    logger.addHandler(console_handler)

    # Add file handler if log_file is specified
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    # Prevent propagation to root logger to avoid duplicate logs
    logger.propagate = False

    return logger


def get_logger(name: str = "rag_pipeline") -> logging.Logger:
    """
    Get a logger instance with the specified name.

    Args:
        name: Name of the logger (default: "rag_pipeline")

    Returns:
        Logger instance
    """
    return logging.getLogger(name)


# Global logger instance for convenience
logger = setup_logger()