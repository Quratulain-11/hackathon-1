from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from uuid import UUID, uuid4


class DocumentSource(BaseModel):
    """
    Represents a source document that can be processed into multiple document chunks.
    """
    id: UUID
    file_path: str
    title: str
    url: Optional[str] = None
    checksum: str
    last_processed_at: Optional[datetime] = None
    chunk_count: int = 0

    class Config:
        # Allow arbitrary types for datetime and UUID
        arbitrary_types_allowed = True

    def __init__(self, **data):
        if 'id' not in data:
            data['id'] = uuid4()
        super().__init__(**data)