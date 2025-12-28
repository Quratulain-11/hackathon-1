from pydantic import BaseModel
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


class Config(BaseModel):
    """
    Configuration class for the RAG pipeline with settings for Cohere, Qdrant, and chunking parameters.
    """
    # Cohere configuration (for embeddings only)
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    embedding_model: str = os.getenv("EMBEDDING_MODEL", "embed-multilingual-v2.0")

    # OpenRouter configuration (for response generation)
    openrouter_api_key: str = os.getenv("OPENROUTER_API_KEY", "")
    openrouter_model: str = os.getenv("OPENROUTER_MODEL", "xiaomi/mimo-v2-flash:free")

    # Qdrant configuration
    qdrant_url: Optional[str] = os.getenv("QDRANT_URL")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
    qdrant_host: str = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port: int = int(os.getenv("QDRANT_PORT", "6333"))
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "docusaurus-content")

    # Chunking configuration
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "512"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "64"))

    # Processing configuration
    max_concurrent_requests: int = int(os.getenv("MAX_CONCURRENT_REQUESTS", "5"))
    request_timeout: int = int(os.getenv("REQUEST_TIMEOUT", "30"))

    # Validation
    class Config:
        # Allow arbitrary types
        arbitrary_types_allowed = True

    def __init__(self, **data):
        super().__init__(**data)

        # Validate required fields
        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable must be set")