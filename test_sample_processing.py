#!/usr/bin/env python3
"""
Test script to verify content extraction works for different Docusaurus formats.
"""
import os
import sys
from pathlib import Path

# Add src to path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from rag_pipeline.config import Config
from rag_pipeline.core.document_processor import DocumentProcessor
from rag_pipeline.core.chunker import Chunker
from rag_pipeline.utils.logger import setup_logger


def test_sample_processing():
    """
    Process sample documents to verify content extraction works for different formats.
    """
    # Set up logging
    logger = setup_logger("test_sample", level="INFO")

    # Load config
    try:
        config = Config()
        logger.info("Configuration loaded successfully")
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        # Create a minimal config for testing
        os.environ["COHERE_API_KEY"] = "test-key"
        config = Config(cohere_api_key="test-key", qdrant_url="http://localhost:6333")

    # Initialize components
    processor = DocumentProcessor(config)
    chunker = Chunker(config)

    # Test directory
    test_dir = "docs/docs"

    if not os.path.exists(test_dir):
        logger.error(f"Test directory {test_dir} does not exist")
        return False

    logger.info(f"Processing documents from {test_dir}")

    # Process the directory
    total_docs = 0
    total_chunks = 0

    for doc_source, content_list in processor.process_directory(test_dir):
        total_docs += 1
        logger.info(f"Processed document: {doc_source.file_path}")
        logger.info(f"  Title: {doc_source.title}")
        logger.info(f"  Checksum: {doc_source.checksum}")

        # Create chunks for this document
        for content in content_list:
            # Prepare metadata for chunking
            metadata = {
                'file_path': doc_source.file_path,
                'page_title': doc_source.title,
                'source_url': doc_source.url,
                'checksum': doc_source.checksum
            }

            # Add content format info if available
            if hasattr(doc_source, 'content_formats'):
                metadata['content_formats'] = doc_source.content_formats

            chunks = chunker.chunk_content(content, metadata)
            total_chunks += len(chunks)

            logger.info(f"  Created {len(chunks)} chunks from document")

            # Log info about first chunk as sample
            if chunks:
                first_chunk = chunks[0]
                logger.info(f"  First chunk size: {len(first_chunk.content)} chars")

                # Count tokens in first chunk
                from rag_pipeline.utils.text_utils import count_tokens
                token_count = count_tokens(first_chunk.content)
                logger.info(f"  First chunk token count: {token_count}")

    logger.info(f"Processing complete. Total documents: {total_docs}, Total chunks: {total_chunks}")

    if total_docs > 0:
        logger.info("✓ Sample processing test PASSED - successfully processed documents with different formats")
        return True
    else:
        logger.error("✗ Sample processing test FAILED - no documents were processed")
        return False


if __name__ == "__main__":
    success = test_sample_processing()
    sys.exit(0 if success else 1)