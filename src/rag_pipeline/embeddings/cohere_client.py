import cohere
from typing import List, Optional
import logging
import time
import random
from ..config import Config


logger = logging.getLogger(__name__)


class CohereClient:
    """
    Wrapper around Cohere client to handle embedding generation.
    """
    def __init__(self, config: Config):
        self.config = config
        # Initialize Cohere client with timeout configuration
        self.client = cohere.Client(
            config.cohere_api_key,
            timeout=200  # 200 seconds timeout as default, will be overridden in embed calls
        )

    def generate_embeddings(self, texts: List[str], max_retries: int = 3) -> Optional[List[List[float]]]:
        """
        Generate embeddings for a list of texts using Cohere with retry logic and timeout handling.

        Args:
            texts: List of text strings to embed
            max_retries: Maximum number of retry attempts

        Returns:
            List of embedding vectors (each is a list of floats), or None if failed
        """
        if not texts:
            logger.warning("No texts provided for embedding generation")
            return None

        # Limit batch size to avoid rate limits (Cohere typically allows up to 96 texts per request)
        if len(texts) > 96:
            logger.warning(f"Batch size {len(texts)} exceeds recommended limit of 96. Consider using batch_generate_embeddings.")

        for attempt in range(max_retries):
            try:
                start_time = time.time()

                # Generate embeddings using Cohere
                response = self.client.embed(
                    texts=texts,
                    model=self.config.embedding_model,
                    input_type="search_document"  # Using search_document as default input type
                )

                # Extract embeddings from response
                embeddings = [embedding for embedding in response.embeddings]
                elapsed_time = (time.time() - start_time) * 1000  # Convert to milliseconds
                logger.info(f"Generated {len(embeddings)} embeddings successfully in {elapsed_time:.2f}ms")

                # Track processing time for performance monitoring (T037)
                avg_time_per_embedding = elapsed_time / len(embeddings) if embeddings else 0
                logger.info(f"Average time per embedding: {avg_time_per_embedding:.2f}ms")

                # Log performance metrics for monitoring (T035)
                if elapsed_time > 200:
                    logger.warning(f"Embedding generation took {elapsed_time:.2f}ms, exceeding 200ms target")

                # Check if we meet the p95 <200ms constraint based on average
                if avg_time_per_embedding > 200:
                    logger.warning(f"Average time per embedding ({avg_time_per_embedding:.2f}ms) exceeds 200ms target")

                # Validate embedding dimensions consistency (T036)
                if not self.validate_embedding_dimensions(embeddings):
                    logger.error("Embedding dimension validation failed")
                    return None

                return embeddings

            except cohere.CohereError as e:
                elapsed_time = (time.time() - start_time) * 1000 if 'start_time' in locals() else 0
                logger.error(f"Cohere API error (attempt {attempt + 1}, {elapsed_time:.2f}ms): {str(e)}")

                # Check if it's a rate limit error
                if "rate limit" in str(e).lower() or "too many requests" in str(e).lower():
                    # Exponential backoff with jitter
                    wait_time = (2 ** attempt) + random.uniform(0, 1)
                    logger.info(f"Rate limited. Waiting {wait_time:.2f}s before retry...")
                    time.sleep(wait_time)
                    continue
                elif "connection" in str(e).lower() or "timeout" in str(e).lower():
                    # Connection or timeout error - wait and retry
                    wait_time = (2 ** attempt) + random.uniform(0, 1)
                    logger.info(f"Connection error. Waiting {wait_time:.2f}s before retry...")
                    time.sleep(wait_time)
                    continue
                else:
                    # Other Cohere-specific error, don't retry
                    logger.error(f"Non-retryable Cohere error: {str(e)}")
                    break
            except Exception as e:
                elapsed_time = (time.time() - start_time) * 1000 if 'start_time' in locals() else 0
                logger.error(f"Unexpected error generating embeddings (attempt {attempt + 1}, {elapsed_time:.2f}ms): {str(e)}")

                if attempt == max_retries - 1:  # Last attempt
                    logger.error("Max retries reached, giving up on embedding generation")
                    break
                else:
                    # Exponential backoff with jitter for general errors
                    wait_time = (2 ** attempt) + random.uniform(0, 1)
                    logger.info(f"Waiting {wait_time:.2f}s before retry...")
                    time.sleep(wait_time)
                    continue

        logger.error("Failed to generate embeddings after all retry attempts")
        return None

    def generate_single_embedding(self, text: str) -> Optional[List[float]]:
        """
        Generate embedding for a single text.

        Args:
            text: Text string to embed

        Returns:
            Embedding vector (list of floats), or None if failed
        """
        try:
            result = self.generate_embeddings([text])
            if result and len(result) > 0:
                return result[0]
            return None
        except Exception as e:
            logger.error(f"Error generating single embedding: {str(e)}")
            return None

    def get_embedding_dimensions(self) -> int:
        """
        Get the expected dimensions for embeddings from the current model.

        Returns:
            Number of dimensions for the embeddings
        """
        # Different Cohere models have different dimensions
        # multilingual models typically have 768 dimensions
        # English models typically have 1024 or 4096 dimensions
        if "multilingual" in self.config.embedding_model.lower():
            return 768  # Updated to correct dimensions for multilingual models
        else:
            return 1024  # Default to 1024 for most Cohere models

    def validate_embedding_dimensions(self, embeddings: List[List[float]], expected_dimensions: int = None) -> bool:
        """
        Validate that all embeddings have consistent dimensions as per data model rules (T036).

        Args:
            embeddings: List of embedding vectors to validate
            expected_dimensions: Expected number of dimensions (uses model default if None)

        Returns:
            True if all embeddings have consistent dimensions, False otherwise
        """
        if expected_dimensions is None:
            expected_dimensions = self.get_embedding_dimensions()

        if not embeddings:
            logger.warning("No embeddings provided for dimension validation")
            return True

        for i, embedding in enumerate(embeddings):
            if len(embedding) != expected_dimensions:
                logger.error(f"Embedding {i} has {len(embedding)} dimensions, expected {expected_dimensions}")
                return False

        logger.info(f"All {len(embeddings)} embeddings have consistent {expected_dimensions}-dimension vectors")
        return True

    def batch_generate_embeddings(self, texts: List[str], batch_size: int = 96) -> Optional[List[List[float]]]:
        """
        Generate embeddings in batches to handle rate limits and large inputs.

        Args:
            texts: List of text strings to embed
            batch_size: Number of texts to process in each batch

        Returns:
            List of embedding vectors (each is a list of floats), or None if failed
        """
        if not texts:
            logger.warning("No texts provided for batch embedding generation")
            return None

        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            try:
                batch_embeddings = self.generate_embeddings(batch)

                if batch_embeddings is None:
                    logger.error(f"Failed to generate embeddings for batch {i//batch_size + 1}")
                    return None

                all_embeddings.extend(batch_embeddings)

                # Add a small delay between batches to respect rate limits
                time.sleep(0.1)

                logger.info(f"Processed batch {i//batch_size + 1}/{(len(texts) - 1)//batch_size + 1}")

            except Exception as e:
                logger.error(f"Error in batch processing: {str(e)}")
                return None

        logger.info(f"Batch processing complete. Generated {len(all_embeddings)} embeddings total")
        return all_embeddings