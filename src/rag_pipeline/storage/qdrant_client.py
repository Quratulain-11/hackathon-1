from qdrant_client import QdrantClient as QdrantBaseClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from typing import List, Dict, Any, Optional
from uuid import UUID
import logging
import os
from ..models.document_chunk import DocumentChunk
from ..config import Config


logger = logging.getLogger(__name__)


class QdrantClient:
    """
    Wrapper around Qdrant client to handle vector storage operations.
    """
    def __init__(self, config: Config):
        self.config = config

        # Initialize Qdrant client
        if config.qdrant_url and config.qdrant_url != "local":
            # Use cloud instance
            self.client = QdrantBaseClient(
                url=config.qdrant_url,
                api_key=config.qdrant_api_key,
                prefer_grpc=True  # Use gRPC for better performance
            )
        else:
            # Use local instance with file storage
            qdrant_path = os.getenv("QDRANT_PATH", "./qdrant_data")
            self.client = QdrantBaseClient(
                path=qdrant_path
            )

    def initialize_collection(self, vector_size: int = 1024) -> bool:
        """
        Initialize the Qdrant collection if it doesn't exist.
        The vector size depends on the embedding model used (e.g., 1024 for multilingual models).

        Args:
            vector_size: Size of the embedding vectors

        Returns:
            True if collection exists or was created successfully
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.config.qdrant_collection_name in collection_names:
                logger.info(f"Collection '{self.config.qdrant_collection_name}' already exists")
                # Get the existing collection info to check vector size
                collection_info = self.client.get_collection(self.config.qdrant_collection_name)
                current_vector_size = collection_info.config.params.vectors.size
                if current_vector_size != vector_size:
                    logger.warning(
                        f"Collection vector size ({current_vector_size}) differs from expected ({vector_size}). "
                        f"This may cause issues with embeddings."
                    )
                return True

            # Create new collection
            self.client.create_collection(
                collection_name=self.config.qdrant_collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
            )

            logger.info(f"Created collection '{self.config.qdrant_collection_name}' with vector size {vector_size}")
            return True

        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {str(e)}")
            # Check if it's a connection error
            if "connection" in str(e).lower() or "connect" in str(e).lower():
                logger.error("Connection error detected - please check Qdrant URL and credentials")
            elif "capacity" in str(e).lower() or "limit" in str(e).lower():
                logger.error("Capacity limit reached - check Qdrant Cloud plan limits")
            return False

    def upsert_embeddings(self, chunks: List[DocumentChunk], embeddings: List[List[float]]) -> bool:
        """
        Upsert document chunks with their embeddings into Qdrant.

        Args:
            chunks: List of DocumentChunk objects
            embeddings: List of embedding vectors (each is a list of floats)

        Returns:
            True if upsert was successful
        """
        if len(chunks) != len(embeddings):
            logger.error(f"Chunk count ({len(chunks)}) doesn't match embedding count ({len(embeddings)})")
            return False

        try:
            # Prepare points for upsert
            points = []
            for chunk, embedding in zip(chunks, embeddings):
                # Create payload with metadata
                payload = {
                    'file_path': chunk.metadata.get('file_path', ''),
                    'page_title': chunk.metadata.get('page_title', ''),
                    'heading': chunk.metadata.get('heading', ''),
                    'section': chunk.metadata.get('section', ''),
                    'chunk_index': chunk.metadata.get('chunk_index', 0),
                    'source_url': chunk.metadata.get('source_url', ''),
                    'checksum': chunk.metadata.get('checksum', ''),
                    'created_at': chunk.created_at.isoformat(),
                    'updated_at': chunk.updated_at.isoformat(),
                }

                # Add any additional metadata fields that might be present
                for key, value in chunk.metadata.items():
                    if key not in payload:
                        payload[key] = value

                point = models.PointStruct(
                    id=str(chunk.id),
                    vector=embedding,
                    payload=payload
                )
                points.append(point)

            # Upsert the points (this handles both insertions and updates)
            self.client.upsert(
                collection_name=self.config.qdrant_collection_name,
                points=points
            )

            logger.info(f"Successfully upserted {len(points)} embeddings to Qdrant")
            return True

        except Exception as e:
            logger.error(f"Error upserting embeddings to Qdrant: {str(e)}")
            # Check if it's a connection error
            if "connection" in str(e).lower() or "connect" in str(e).lower():
                logger.error("Connection error detected during upsert - please check Qdrant connection")
            elif "capacity" in str(e).lower() or "limit" in str(e).lower():
                logger.error("Capacity limit reached during upsert - check Qdrant Cloud plan limits")
            elif "timeout" in str(e).lower():
                logger.error("Timeout error during upsert - consider reducing batch size")
            return False

    def update_embeddings_for_document(self, file_path: str, chunks: List[DocumentChunk], embeddings: List[List[float]]) -> bool:
        """
        Update embeddings for a specific document by deleting old embeddings and adding new ones.
        This handles the case where document content changes.

        Args:
            file_path: Path of the document to update
            chunks: New document chunks
            embeddings: New embedding vectors

        Returns:
            True if update was successful
        """
        try:
            # First, delete existing embeddings for this document
            self.delete_embeddings_by_file_path(file_path)

            # Then upsert the new embeddings
            success = self.upsert_embeddings(chunks, embeddings)

            if success:
                logger.info(f"Successfully updated embeddings for document: {file_path}")
            else:
                logger.error(f"Failed to update embeddings for document: {file_path}")

            return success

        except Exception as e:
            logger.error(f"Error updating embeddings for document {file_path}: {str(e)}")
            return False

    def delete_embeddings_by_file_path(self, file_path: str) -> bool:
        """
        Delete all embeddings associated with a specific file path.

        Args:
            file_path: Path of the file whose embeddings should be deleted

        Returns:
            True if deletion was successful
        """
        try:
            # Find points with matching file_path
            search_result = self.client.scroll(
                collection_name=self.config.qdrant_collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="file_path",
                            match=models.MatchValue(value=file_path)
                        )
                    ]
                ),
                limit=10000  # Assuming we don't have more than 10k chunks per file
            )

            # Extract IDs of points to delete
            point_ids = [point.id for point in search_result[0]]

            if point_ids:
                # Delete the points
                self.client.delete(
                    collection_name=self.config.qdrant_collection_name,
                    points_selector=models.PointIdsList(
                        points=point_ids
                    )
                )
                logger.info(f"Deleted {len(point_ids)} embeddings for file: {file_path}")
            else:
                logger.info(f"No existing embeddings found for file: {file_path}")

            return True

        except Exception as e:
            logger.error(f"Error deleting embeddings for file {file_path}: {str(e)}")
            return False

    def search_similar(self, query_embedding: List[float], top_k: int = 5, filters: Optional[Dict] = None) -> List[Dict]:
        """
        Search for similar embeddings in Qdrant.

        Args:
            query_embedding: The embedding vector to search for similarity
            top_k: Number of results to return
            filters: Optional filters to apply to the search

        Returns:
            List of similar chunks with scores
        """
        try:
            # Prepare filters if provided
            search_filter = None
            if filters:
                conditions = []
                for key, value in filters.items():
                    conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )

                if conditions:
                    search_filter = models.Filter(
                        must=conditions
                    )

            # Perform search using the correct method for vector search
            # For local Qdrant client with path=, the interface might be different
            # Check if search_points method exists (this is the proper method for local client vector search)
            if hasattr(self.client, 'search_points'):
                # Use search_points for local Qdrant client
                results = self.client.search_points(
                    collection_name=self.config.qdrant_collection_name,
                    vector=query_embedding,
                    filter=search_filter,
                    limit=top_k
                )
            else:
                # For other clients, try the standard search method
                try:
                    results = self.client.search(
                        collection_name=self.config.qdrant_collection_name,
                        query_vector=query_embedding,
                        query_filter=search_filter,
                        limit=top_k
                    )
                except AttributeError:
                    # If search method doesn't exist, try query method (for fastembed local client)
                    # But format the call properly to avoid the positional argument issue
                    try:
                        results = self.client.query(
                            collection_name=self.config.qdrant_collection_name,
                            query=query_embedding,
                            query_filter=search_filter,
                            limit=top_k
                        )
                    except Exception as e:
                        logger.error(f"Error searching in Qdrant: {str(e)}")
                        results = []

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    'chunk_id': result.id,
                    'content': result.payload.get('content', ''),
                    'score': result.score,
                    'metadata': {k: v for k, v in result.payload.items() if k != 'content'},
                    'vector': result.vector
                })

            logger.info(f"Search returned {len(formatted_results)} results")
            return formatted_results

        except Exception as e:
            logger.error(f"Error searching in Qdrant: {str(e)}")
            # Check if it's a connection error
            if "connection" in str(e).lower() or "connect" in str(e).lower():
                logger.error("Connection error detected during search - please check Qdrant connection")
            elif "capacity" in str(e).lower() or "limit" in str(e).lower():
                logger.error("Capacity limit reached during search - check Qdrant Cloud plan limits")
            elif "timeout" in str(e).lower():
                logger.error("Timeout error during search - consider reducing query complexity")
            return []

    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict]:
        """
        Retrieve a specific chunk by its ID.

        Args:
            chunk_id: The ID of the chunk to retrieve

        Returns:
            Chunk data if found, None otherwise
        """
        try:
            records = self.client.retrieve(
                collection_name=self.config.qdrant_collection_name,
                ids=[chunk_id]
            )

            if records:
                record = records[0]
                return {
                    'chunk_id': record.id,
                    'content': record.payload.get('content', ''),
                    'metadata': {k: v for k, v in record.payload.items() if k != 'content'},
                    'vector': record.vector
                }

            return None

        except Exception as e:
            logger.error(f"Error retrieving chunk {chunk_id} from Qdrant: {str(e)}")
            return None

    def delete_collection(self) -> bool:
        """
        Delete the entire collection (use with caution!).

        Returns:
            True if deletion was successful
        """
        try:
            self.client.delete_collection(self.config.qdrant_collection_name)
            logger.info(f"Deleted collection '{self.config.qdrant_collection_name}'")
            return True
        except Exception as e:
            logger.error(f"Error deleting collection: {str(e)}")
            return False

    def get_collection_info(self) -> Optional[Dict]:
        """
        Get information about the current collection.

        Returns:
            Collection info if available, None otherwise
        """
        try:
            collection_info = self.client.get_collection(self.config.qdrant_collection_name)
            return {
                'name': self.config.qdrant_collection_name,  # Use config name since API doesn't return it
                'vector_size': collection_info.config.params.vectors.size,
                'distance': collection_info.config.params.vectors.distance,
                'count': collection_info.points_count
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}")
            return None