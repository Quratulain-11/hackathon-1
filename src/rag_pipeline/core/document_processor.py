from pathlib import Path
from typing import List, Generator, Tuple, Dict
from ..models.document_chunk import DocumentChunk
from ..models.document_source import DocumentSource
from ..utils.file_utils import find_markdown_files, read_file_content, get_file_checksum
from ..utils.text_utils import clean_text_for_chunking
from ..core.metadata_extractor import extract_document_metadata
from ..utils.logger import get_logger
from ..config import Config
import uuid
from datetime import datetime


logger = get_logger(__name__)


class DocumentProcessor:
    """
    Processes Docusaurus documentation files to extract content and prepare for embedding.
    """
    def __init__(self, config: Config):
        self.config = config

    def process_document(self, file_path: str) -> Tuple[DocumentSource, List[str]]:
        """
        Process a single document file to extract content and metadata.

        Args:
            file_path: Path to the document file

        Returns:
            Tuple of (DocumentSource, list of cleaned text chunks)
        """
        logger.info(f"Processing document: {file_path}")

        # Read raw content
        raw_content = read_file_content(file_path)

        # Calculate checksum
        checksum = get_file_checksum(file_path)

        # Extract metadata
        metadata = extract_document_metadata(raw_content, file_path)

        # Enhance metadata with content format information
        metadata = self._enhance_metadata_with_content_info(metadata, raw_content)

        # Clean the content for processing
        cleaned_content = clean_text_for_chunking(raw_content)

        # Additional cleaning specific to Docusaurus content
        cleaned_content = self._clean_docusaurus_specific_content(cleaned_content)

        # Create DocumentSource
        document_source = DocumentSource(
            file_path=file_path,
            title=metadata.get('title', Path(file_path).stem),
            url=metadata.get('url', None),
            checksum=checksum,
            last_processed_at=datetime.now(),
            chunk_count=0  # Will be updated when chunks are created
        )

        # Return the source and the cleaned content for further processing
        return document_source, [cleaned_content]

    def _clean_docusaurus_specific_content(self, content: str) -> str:
        """
        Clean Docusaurus-specific content like admonitions, tabs, etc.

        Args:
            content: Raw content to clean

        Returns:
            Cleaned content
        """
        import re

        # Remove Docusaurus admonitions markers (e.g., :::info, :::caution, :::danger)
        content = re.sub(r'::{3}[\w-]+\n?', '', content)
        content = re.sub(r'::{3}', '', content)

        # Remove Docusaurus tab markers
        content = re.sub(r'<Tabs[^>]*>', '', content)
        content = re.sub(r'</Tabs>', '', content)
        content = re.sub(r'<TabItem[^>]*>', '', content)
        content = re.sub(r'</TabItem>', '', content)

        # Remove HTML comments
        content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)

        # Normalize multiple newlines
        content = re.sub(r'\n\s*\n\s*\n+', '\n\n', content)

        return content.strip()

    def _enhance_metadata_with_content_info(self, metadata: Dict, content: str) -> Dict:
        """
        Enhance metadata with information about content formats present in the document.

        Args:
            metadata: Existing metadata dictionary
            content: Raw content to analyze

        Returns:
            Enhanced metadata dictionary
        """
        import re

        # Detect content formats
        has_code_blocks = bool(re.search(r'```[\s\S]*?```', content))
        has_headers = bool(re.search(r'^#+\s+', content, re.MULTILINE))
        has_lists = bool(re.search(r'^\s*[\*\-\+]\s+|^(\d+\.){1,2}\s+', content, re.MULTILINE))
        has_links = bool(re.search(r'\[([^\]]+)\]\([^)]+\)', content))
        has_images = bool(re.search(r'!\[([^\]]*)\]\([^)]+\)', content))
        has_bold = bool(re.search(r'\*{2}[^*]+\*{2}|_{2}[^_]+_{2}', content))
        has_italic = bool(re.search(r'\*[^*]+\*|_[^_]+_', content))
        has_blockquotes = bool(re.search(r'^\s*>\s+', content, re.MULTILINE))

        # Docusaurus-specific formats
        has_admonitions = bool(re.search(r'::{3}[\w-]+', content))
        has_tabs = bool(re.search(r'<Tabs[^>]*>|<TabItem[^>]*>', content, re.IGNORECASE))
        has_docusaurus_links = 'doc:' in content or '{@' in content  # Docusaurus link formats

        # Add format information to metadata
        metadata['content_formats'] = {
            'has_code_blocks': has_code_blocks,
            'has_headers': has_headers,
            'has_lists': has_lists,
            'has_links': has_links,
            'has_images': has_images,
            'has_bold': has_bold,
            'has_italic': has_italic,
            'has_blockquotes': has_blockquotes,
            'has_admonitions': has_admonitions,
            'has_tabs': has_tabs,
            'has_docusaurus_links': has_docusaurus_links
        }

        return metadata

    def process_directory(self, directory_path: str) -> Generator[Tuple[DocumentSource, List[str]], None, None]:
        """
        Process all markdown files in a directory.

        Args:
            directory_path: Path to the directory to process

        Yields:
            Tuples of (DocumentSource, list of cleaned text chunks)
        """
        logger.info(f"Processing directory: {directory_path}")

        # Find all markdown files
        markdown_files = find_markdown_files(directory_path)

        processed_count = 0
        for file_path in markdown_files:
            try:
                # Process each file
                document_source, content_chunks = self.process_document(str(file_path))
                yield document_source, content_chunks
                processed_count += 1

                if processed_count % 10 == 0:  # Log progress every 10 files
                    logger.info(f"Processed {processed_count} files...")

            except Exception as e:
                logger.error(f"Error processing file {file_path}: {str(e)}")
                continue

        logger.info(f"Completed processing directory: {directory_path}, total files processed: {processed_count}")

    def process_specific_files(self, file_paths: List[str]) -> Generator[Tuple[DocumentSource, List[str]], None, None]:
        """
        Process specific files provided in a list.

        Args:
            file_paths: List of file paths to process

        Yields:
            Tuples of (DocumentSource, list of cleaned text chunks)
        """
        for file_path in file_paths:
            try:
                document_source, content_chunks = self.process_document(file_path)
                yield document_source, content_chunks
            except Exception as e:
                logger.error(f"Error processing file {file_path}: {str(e)}")
                continue