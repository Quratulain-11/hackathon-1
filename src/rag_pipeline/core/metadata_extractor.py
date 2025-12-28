import re
from pathlib import Path
from typing import Dict, Optional, Tuple
import frontmatter  # For parsing YAML frontmatter
from bs4 import BeautifulSoup
import logging

logger = logging.getLogger(__name__)


def extract_frontmatter(content: str) -> Tuple[str, Dict]:
    """
    Extract YAML frontmatter from markdown content and return the remaining content.

    Args:
        content: Raw markdown content

    Returns:
        Tuple of (content without frontmatter, frontmatter data as dict)
    """
    try:
        # Use frontmatter to parse the content
        # Using the available Frontmatter.read method from the installed package
        fm = frontmatter.Frontmatter()
        result = fm.read(content)
        return result['body'], result['attributes']
    except Exception as e:
        logger.warning(f"Error parsing frontmatter: {str(e)}. Returning original content.")
        return content, {}


def extract_title_from_content(content: str, frontmatter_data: Dict = None) -> Optional[str]:
    """
    Extract the title from markdown content, prioritizing frontmatter if available.

    Args:
        content: Markdown content without frontmatter
        frontmatter_data: Frontmatter data extracted from the document

    Returns:
        Extracted title or None if not found
    """
    # First, try to get title from frontmatter
    if frontmatter_data and frontmatter_data.get('title'):
        return frontmatter_data['title']

    # If no title in frontmatter, try to extract from H1 in content
    lines = content.split('\n')
    for line in lines:
        # Look for H1 markdown format: # Title
        h1_match = re.match(r'^#\s+(.+)', line.strip())
        if h1_match:
            return h1_match.group(1).strip()

        # Look for H1 HTML format: <h1>Title</h1>
        h1_html_match = re.search(r'<h1[^>]*>(.*?)</h1>', line, re.IGNORECASE)
        if h1_html_match:
            # Remove HTML tags from the title
            soup = BeautifulSoup(f"<div>{h1_html_match.group(1)}</div>", 'html.parser')
            return soup.get_text().strip()

    return None


def extract_headings(content: str) -> list:
    """
    Extract all headings from markdown content.

    Args:
        content: Markdown content

    Returns:
        List of headings found in the content
    """
    headings = []
    lines = content.split('\n')

    for line in lines:
        # Match markdown headings: ## Heading, ### Heading, etc.
        heading_match = re.match(r'^(#+)\s+(.+)', line.strip())
        if heading_match:
            level = len(heading_match.group(1))
            text = heading_match.group(2).strip()
            headings.append({
                'level': level,
                'text': text
            })

    return headings


def extract_metadata_from_path(file_path: str) -> Dict[str, str]:
    """
    Extract metadata from the file path structure.

    Args:
        file_path: Path to the markdown file

    Returns:
        Dictionary with path-based metadata
    """
    path_obj = Path(file_path)
    metadata = {
        'file_path': str(path_obj),
        'file_name': path_obj.name,
        'file_stem': path_obj.stem,
        'file_extension': path_obj.suffix,
        'directory': str(path_obj.parent),
    }

    # Extract module information from path structure (e.g., docs/docs/module-name/file.md)
    parts = path_obj.parts
    for i, part in enumerate(parts):
        if 'docs' in part and i < len(parts) - 1:
            # Next part might be the module name
            if i + 1 < len(parts):
                metadata['module'] = parts[i + 1]
                break

    return metadata


def extract_document_metadata(content: str, file_path: str) -> Dict:
    """
    Extract comprehensive metadata from document content and path.

    Args:
        content: Raw markdown content
        file_path: Path to the document

    Returns:
        Dictionary with all extracted metadata
    """
    # Extract frontmatter
    content_without_frontmatter, frontmatter_data = extract_frontmatter(content)

    # Extract title
    title = extract_title_from_content(content_without_frontmatter, frontmatter_data)

    # Extract path-based metadata
    path_metadata = extract_metadata_from_path(file_path)

    # Extract headings
    headings = extract_headings(content_without_frontmatter)

    # Combine all metadata
    metadata = {
        'title': title,
        'frontmatter': frontmatter_data,
        'path_info': path_metadata,
        'headings': headings,
        'has_frontmatter': bool(frontmatter_data)
    }

    # Add any frontmatter fields directly to top level if they don't conflict
    for key, value in frontmatter_data.items():
        if key not in metadata:
            metadata[key] = value

    return metadata