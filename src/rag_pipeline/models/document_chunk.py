from pydantic import BaseModel
from typing import Optional, Dict, Any, List
from datetime import datetime
from uuid import UUID, uuid4


class DocumentChunk(BaseModel):
    """
    Represents a chunk of processed document content with its embedding and metadata.
    """
    id: UUID
    content: str
    embedding: Optional[List[float]] = None
    metadata: Dict[str, Any]  # Contains file_path, page_title, heading, section, chunk_index, source_url
    created_at: datetime
    updated_at: datetime

    class Config:
        # Allow arbitrary types for datetime and UUID
        arbitrary_types_allowed = True

    def __init__(self, **data):
        if 'id' not in data:
            data['id'] = uuid4()
        if 'created_at' not in data:
            data['created_at'] = datetime.now()
        if 'updated_at' not in data:
            data['updated_at'] = datetime.now()
        super().__init__(**data)