import click
import os
import sys
from pathlib import Path
from typing import Optional

# Add the src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from ..config import Config
from ..core.document_processor import DocumentProcessor
from ..core.chunker import Chunker
from ..core.embedding_pipeline import EmbeddingPipeline
from ..utils.logger import setup_logger
from ..storage.qdrant_client import QdrantClient
from ..embeddings.cohere_client import CohereClient


@click.group()
def main():
    """RAG Pipeline CLI for processing Docusaurus documentation and generating embeddings."""
    pass


@main.command()
@click.option('--docs-path', default='docs/docs', help='Path to documentation directory')
@click.option('--chunk-size', default=512, help='Maximum tokens per chunk')
@click.option('--overlap', default=64, help='Token overlap between chunks')
@click.option('--collection-name', default='docusaurus-content', help='Qdrant collection name')
@click.option('--model', default='embed-multilingual-v2.0', help='Cohere embedding model to use')
@click.option('--verbose', '-v', is_flag=True, help='Enable verbose logging')
def process(docs_path: str, chunk_size: int, overlap: int, collection_name: str, model: str, verbose: bool):
    """Process documentation and generate embeddings."""
    # Set up logging
    log_level = "DEBUG" if verbose else "INFO"
    logger = setup_logger("rag_pipeline", level=log_level)

    logger.info(f"Starting RAG pipeline process for directory: {docs_path}")

    # Check if docs path exists
    if not os.path.exists(docs_path):
        logger.error(f"Documentation path does not exist: {docs_path}")
        return

    # Load configuration
    try:
        config = Config(
            chunk_size=chunk_size,
            chunk_overlap=overlap,
            qdrant_collection_name=collection_name,
            embedding_model=model
        )
        logger.info("Configuration loaded successfully")
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        return

    # Initialize the complete embedding pipeline
    pipeline = EmbeddingPipeline(config)

    # Process and store documents in Qdrant
    try:
        success = pipeline.process_and_store_directory(docs_path)
        if success:
            logger.info("Processing and storage completed successfully")
        else:
            logger.error("Processing and storage failed")
            return
    except Exception as e:
        logger.error(f"Error during processing: {e}")
        return


@main.command()
@click.option('--docs-path', default='docs/docs', help='Path to documentation directory')
@click.option('--chunk-size', default=512, help='Maximum tokens per chunk')
@click.option('--overlap', default=64, help='Token overlap between chunks')
@click.option('--collection-name', default='docusaurus-content', help='Qdrant collection name')
@click.option('--model', default='embed-multilingual-v2.0', help='Cohere embedding model to use')
@click.option('--verbose', '-v', is_flag=True, help='Enable verbose logging')
def test_run(docs_path: str, chunk_size: int, overlap: int, collection_name: str, model: str, verbose: bool):
    """Test run - process a few documents to verify the pipeline works."""
    # Set up logging
    log_level = "DEBUG" if verbose else "INFO"
    logger = setup_logger("rag_pipeline", level=log_level)

    logger.info(f"Starting test run for directory: {docs_path}")

    # Check if docs path exists
    if not os.path.exists(docs_path):
        logger.error(f"Documentation path does not exist: {docs_path}")
        return

    # Load configuration
    try:
        config = Config(
            chunk_size=chunk_size,
            chunk_overlap=overlap,
            qdrant_collection_name=collection_name,
            embedding_model=model
        )
        logger.info("Configuration loaded successfully")
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        return

    # Initialize components
    processor = DocumentProcessor(config)
    chunker = Chunker(config)

    # Process only the first few documents as a test
    logger.info("Starting test document processing...")
    processed_count = 0

    try:
        for doc_source, content_list in processor.process_directory(docs_path):
            processed_count += 1
            logger.info(f"Test processing document: {doc_source.file_path}")

            # Process each content chunk in the document
            for content in content_list:
                # Prepare metadata for chunking
                metadata = {
                    'file_path': doc_source.file_path,
                    'page_title': doc_source.title,
                    'source_url': doc_source.url,
                    'checksum': doc_source.checksum
                }

                # Create chunks for this content
                chunks = chunker.chunk_content(content, metadata)

                logger.info(f"Created {len(chunks)} chunks for {doc_source.file_path}")

                # Just validate the chunks are created properly (don't store them in this test)
                for i, chunk in enumerate(chunks[:2]):  # Only check first 2 chunks as sample
                    logger.debug(f"  Chunk {i+1}: {len(chunk.content)} chars, {chunk.metadata.get('chunk_index', 'N/A')} index")

            # Only process first 3 documents for the test
            if processed_count >= 3:
                break

        logger.info(f"Test run complete! Processed {processed_count} documents")

    except Exception as e:
        logger.error(f"Error during test processing: {e}")
        return


if __name__ == "__main__":
    main()