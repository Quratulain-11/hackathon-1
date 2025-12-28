from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from uuid import UUID


class EmbeddingResult(BaseModel):
    """
    Represents the result of an embedding generation operation for a document chunk.
    """
    document_id: UUID
    chunk_id: UUID
    embedding_model: str
    embedding_dimensions: int
    processing_time: float  # in seconds
    status: str  # "success", "failed", "pending"
    error_message: Optional[str] = None
    created_at: datetime = datetime.now()

    class Config:
        # Allow arbitrary types for datetime and UUID
        arbitrary_types_allowed = True