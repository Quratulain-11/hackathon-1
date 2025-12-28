import re
from typing import List, Tuple
import frontmatter
import logging

logger = logging.getLogger(__name__)


def remove_frontmatter(content: str) -> str:
    """
    Remove YAML frontmatter from markdown content.

    Args:
        content: Raw markdown content with potential frontmatter

    Returns:
        Content without frontmatter
    """
    try:
        # Use frontmatter to parse and remove the frontmatter section
        # Using the available Frontmatter.read method from the installed package
        fm = frontmatter.Frontmatter()
        result = fm.read(content)
        return result['body']
    except Exception as e:
        logger.warning(f"Error parsing frontmatter: {str(e)}. Returning original content.")
        return content


def clean_markdown_content(content: str) -> str:
    """
    Clean markdown content by removing unnecessary whitespace and special characters.

    Args:
        content: Raw markdown content

    Returns:
        Cleaned content
    """
    # Remove leading/trailing whitespace
    content = content.strip()

    # Normalize whitespace - replace multiple spaces with single space
    content = re.sub(r'\s+', ' ', content)

    # Remove unnecessary newlines but preserve paragraph structure
    content = re.sub(r'\n\s*\n', '\n\n', content)

    # Remove special characters that might interfere with processing
    # Keep letters, numbers, punctuation, and common symbols
    # content = re.sub(r'[^\w\s\-\.\,\!\?\;\:\'"()\[\]{}#@$/\\%&*+=<>~^|`]', ' ', content)

    return content


def extract_text_from_markdown(markdown_content: str) -> str:
    """
    Extract plain text from markdown content by removing markdown formatting.

    Args:
        markdown_content: Raw markdown content

    Returns:
        Plain text content
    """
    text = markdown_content

    # Remove markdown headers (# Header)
    text = re.sub(r'^#+\s+', '', text, flags=re.MULTILINE)

    # Remove emphasis (*italic*, **bold**, _italic_, __bold__)
    text = re.sub(r'\*{1,2}([^*]+)\*{1,2}', r'\1', text)
    text = re.sub(r'_{1,2}([^_]+)_{1,2}', r'\1', text)

    # Remove inline code (`code`)
    text = re.sub(r'`([^`]+)`', r'\1', text)

    # Remove code blocks (```code```)
    text = re.sub(r'```.*?\n(.*?)```', r'\1', text, flags=re.DOTALL)

    # Remove links [text](url) - keep the text part
    text = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text)

    # Remove images ![alt](url) - replace with alt text
    text = re.sub(r'!\[([^\]]*)\]\([^)]+\)', r'\1', text)

    # Remove horizontal rules
    text = re.sub(r'^\s*[-*_]{3,}\s*$', '', text, flags=re.MULTILINE)

    # Clean up extra whitespace
    text = clean_markdown_content(text)

    return text


def normalize_whitespace(text: str) -> str:
    """
    Normalize whitespace in text by converting various whitespace characters to standard spaces.

    Args:
        text: Input text

    Returns:
        Text with normalized whitespace
    """
    # Replace various whitespace characters with standard space
    text = re.sub(r'[\t\r\n\f\v]+', ' ', text)

    # Normalize multiple spaces to single space
    text = re.sub(r' +', ' ', text)

    return text.strip()


def remove_special_characters(text: str, keep_punctuation: bool = True) -> str:
    """
    Remove special characters from text.

    Args:
        text: Input text
        keep_punctuation: Whether to keep common punctuation marks

    Returns:
        Text with special characters removed
    """
    if keep_punctuation:
        # Keep letters, numbers, common punctuation, and spaces
        cleaned_text = re.sub(r'[^\w\s\-\.\,\!\?\;\:\'"()\[\]{}#@$/\\%&*+=<>~^|`]', ' ', text)
    else:
        # Keep only letters, numbers, and spaces
        cleaned_text = re.sub(r'[^\w\s]', ' ', text)

    # Normalize whitespace after character removal
    return normalize_whitespace(cleaned_text)


def clean_text_for_chunking(content: str) -> str:
    """
    Comprehensive text cleaning specifically for chunking preparation.

    Args:
        content: Raw content to clean

    Returns:
        Cleaned content ready for chunking
    """
    # First, remove frontmatter if present
    content = remove_frontmatter(content)

    # Extract plain text from markdown
    content = extract_text_from_markdown(content)

    # Normalize whitespace
    content = normalize_whitespace(content)

    # Clean content
    content = clean_markdown_content(content)

    return content


def count_tokens(text: str) -> int:
    """
    Estimate the number of tokens in text (for rough token counting).
    This is a simple approximation - for precise token counting, use a tokenizer from the specific model.

    Args:
        text: Input text

    Returns:
        Estimated number of tokens
    """
    # Simple tokenization: split on whitespace and punctuation
    # This is a rough approximation; for accurate counting, use model-specific tokenizers
    import re
    tokens = re.findall(r'\b\w+\b|[^\w\s]', text)
    return len(tokens)