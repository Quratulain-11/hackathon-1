"""
Pipeline for processing documents, chunking them, and generating embeddings with Cohere.
"""
from typing import List, Generator, Tuple
from ..models.document_chunk import DocumentChunk
from ..models.document_source import DocumentSource
from .document_processor import DocumentProcessor
from .chunker import Chunker
from ..embeddings.cohere_client import CohereClient
from ..storage.qdrant_client import QdrantClient
from ..config import Config
from ..utils.logger import get_logger


logger = get_logger(__name__)


class EmbeddingPipeline:
    """
    Coordinates the complete pipeline: document processing -> chunking -> embedding generation -> storage.
    """
    def __init__(self, config: Config):
        self.config = config
        self.document_processor = DocumentProcessor(config)
        self.chunker = Chunker(config)
        self.cohere_client = CohereClient(config)
        self.qdrant_client = QdrantClient(config)

    def process_document_for_embeddings(self, file_path: str) -> Tuple[List[DocumentChunk], List[List[float]]]:
        """
        Process a single document through the complete pipeline: process -> chunk -> embed.

        Args:
            file_path: Path to the document file

        Returns:
            Tuple of (DocumentChunks, embedding vectors)
        """
        logger.info(f"Processing document for embeddings: {file_path}")

        # Process the document to get source and content
        doc_source, content_list = self.document_processor.process_document(file_path)

        all_chunks = []
        all_embeddings = []

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
            chunks = self.chunker.chunk_content(content, metadata)
            all_chunks.extend(chunks)

            logger.info(f"Created {len(chunks)} chunks for {file_path}")

            # Generate embeddings for chunks
            if chunks:
                chunk_texts = [chunk.content for chunk in chunks]
                embeddings = self.cohere_client.generate_embeddings(chunk_texts)

                if embeddings:
                    # Validate embedding dimensions
                    if not self.cohere_client.validate_embedding_dimensions(embeddings):
                        logger.error(f"Embedding dimension validation failed for {file_path}")
                        return [], []

                    all_embeddings.extend(embeddings)
                    logger.info(f"Generated {len(embeddings)} embeddings for {file_path}")
                else:
                    logger.error(f"Failed to generate embeddings for {file_path}")
                    return [], []

        logger.info(f"Completed processing {file_path}: {len(all_chunks)} chunks, {len(all_embeddings)} embeddings")
        return all_chunks, all_embeddings

    def process_directory_for_embeddings(self, directory_path: str) -> Generator[Tuple[List[DocumentChunk], List[List[float]]], None, None]:
        """
        Process all documents in a directory through the complete pipeline.

        Args:
            directory_path: Path to the directory to process

        Yields:
            Tuples of (DocumentChunks, embedding vectors) for each processed document
        """
        logger.info(f"Starting directory processing for embeddings: {directory_path}")

        processed_count = 0
        for doc_source, content_list in self.document_processor.process_directory(directory_path):
            try:
                all_chunks = []
                all_embeddings = []

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
                    chunks = self.chunker.chunk_content(content, metadata)
                    all_chunks.extend(chunks)

                    logger.info(f"Created {len(chunks)} chunks for {doc_source.file_path}")

                # Generate embeddings for all chunks of this document
                if all_chunks:
                    chunk_texts = [chunk.content for chunk in all_chunks]
                    embeddings = self.cohere_client.generate_embeddings(chunk_texts)

                    if embeddings:
                        # Validate embedding dimensions
                        if not self.cohere_client.validate_embedding_dimensions(embeddings):
                            logger.error(f"Embedding dimension validation failed for {doc_source.file_path}")
                            continue  # Skip this document

                        all_embeddings.extend(embeddings)
                        logger.info(f"Generated {len(embeddings)} embeddings for {doc_source.file_path}")
                    else:
                        logger.error(f"Failed to generate embeddings for {doc_source.file_path}")
                        continue  # Skip this document

                yield all_chunks, all_embeddings
                processed_count += 1

                if processed_count % 10 == 0:  # Log progress every 10 documents
                    logger.info(f"Processed {processed_count} documents...")

            except Exception as e:
                logger.error(f"Error processing document {doc_source.file_path}: {str(e)}")
                continue

        logger.info(f"Completed directory processing: {processed_count} documents processed")

    def process_and_store_directory(self, directory_path: str) -> bool:
        """
        Process all documents in a directory and store embeddings in Qdrant.

        Args:
            directory_path: Path to the directory to process

        Returns:
            True if processing and storage were successful
        """
        logger.info(f"Starting process and store for directory: {directory_path}")

        # Initialize Qdrant collection with correct vector size based on embedding model
        vector_size = self.cohere_client.get_embedding_dimensions()
        if not self.qdrant_client.initialize_collection(vector_size=vector_size):
            logger.error("Failed to initialize Qdrant collection")
            return False

        total_chunks = 0
        total_documents = 0

        try:
            for chunks, embeddings in self.process_directory_for_embeddings(directory_path):
                if chunks and embeddings:
                    # Store embeddings in Qdrant
                    success = self.qdrant_client.upsert_embeddings(chunks, embeddings)
                    if success:
                        logger.info(f"Stored {len(chunks)} embeddings in Qdrant")
                        total_chunks += len(chunks)
                        total_documents += 1
                    else:
                        logger.error(f"Failed to store embeddings for {len(chunks)} chunks in Qdrant")
                        return False
                else:
                    logger.warning("No chunks or embeddings generated for document, skipping storage")

            logger.info(f"Completed process and store: {total_documents} documents, {total_chunks} chunks stored")
            return True

        except Exception as e:
            logger.error(f"Error during process and store: {str(e)}")
            return False