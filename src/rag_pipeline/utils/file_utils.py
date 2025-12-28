import os
from pathlib import Path
from typing import List, Generator
import logging

logger = logging.getLogger(__name__)


def find_markdown_files(directory: str, extensions: List[str] = None) -> Generator[Path, None, None]:
    """
    Recursively find all markdown files in a directory.

    Args:
        directory: The directory to search in
        extensions: List of file extensions to look for (default: ['.md', '.mdx'])

    Returns:
        Generator of Path objects for markdown files
    """
    if extensions is None:
        extensions = ['.md', '.mdx']

    directory_path = Path(directory)

    if not directory_path.exists():
        logger.error(f"Directory does not exist: {directory}")
        return

    for file_path in directory_path.rglob('*'):
        if file_path.suffix.lower() in extensions:
            yield file_path


def read_file_content(file_path: str) -> str:
    """
    Read the content of a file.

    Args:
        file_path: Path to the file to read

    Returns:
        Content of the file as a string
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            return file.read()
    except UnicodeDecodeError:
        # Try with different encoding if utf-8 fails
        with open(file_path, 'r', encoding='latin-1') as file:
            return file.read()
    except Exception as e:
        logger.error(f"Error reading file {file_path}: {str(e)}")
        raise


def get_relative_path(file_path: str, base_directory: str) -> str:
    """
    Get the relative path of a file with respect to a base directory.

    Args:
        file_path: The absolute path of the file
        base_directory: The base directory to calculate relative path from

    Returns:
        Relative path as a string
    """
    file_path_obj = Path(file_path)
    base_path_obj = Path(base_directory)

    try:
        relative_path = file_path_obj.relative_to(base_path_obj)
        return str(relative_path)
    except ValueError:
        # If file_path is not within base_directory, return the original path
        return file_path


def ensure_directory_exists(directory_path: str) -> bool:
    """
    Ensure that a directory exists, creating it if necessary.

    Args:
        directory_path: Path to the directory

    Returns:
        True if directory exists or was created successfully
    """
    try:
        Path(directory_path).mkdir(parents=True, exist_ok=True)
        return True
    except Exception as e:
        logger.error(f"Error creating directory {directory_path}: {str(e)}")
        return False


def get_file_checksum(file_path: str) -> str:
    """
    Generate a checksum for a file's content.

    Args:
        file_path: Path to the file

    Returns:
        MD5 checksum as a hex string
    """
    import hashlib

    hash_md5 = hashlib.md5()
    try:
        with open(file_path, "rb") as f:
            # Read file in chunks to handle large files efficiently
            for chunk in iter(lambda: f.read(4096), b""):
                hash_md5.update(chunk)
        return hash_md5.hexdigest()
    except Exception as e:
        logger.error(f"Error calculating checksum for {file_path}: {str(e)}")
        raise