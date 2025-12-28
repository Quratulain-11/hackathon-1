"""
Pydantic models for chat response in the RAG Chatbot API.
"""
from pydantic import BaseModel, validator
from typing import List, Optional, Dict, Any
from datetime import datetime


class DocumentSource(BaseModel):
    """
    Represents a document chunk used as source material for the response.
    """
    content: str
    file_path: str
    page_title: str
    heading: Optional[str] = None
    relevance_score: float
    chunk_index: Optional[int] = None

    @validator('content')
    def validate_content(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('content must not be empty')
        return v

    @validator('file_path')
    def validate_file_path(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('file_path must not be empty')
        return v

    @validator('relevance_score')
    def validate_relevance_score(cls, v):
        if v is not None and (v < 0.0 or v > 1.0):
            raise ValueError('relevance_score must be between 0.0 and 1.0')
        return v


class ChatResponse(BaseModel):
    """
    Represents the AI-generated response to a user query.
    """
    answer: str
    sources: List[DocumentSource]
    query: str
    confidence: float
    retrieval_time_ms: float
    response_time_ms: float
    timestamp: datetime

    @validator('answer')
    def validate_answer(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('answer must not be empty')
        return v

    @validator('confidence')
    def validate_confidence(cls, v):
        if v is not None and (v < 0.0 or v > 1.0):
            raise ValueError('confidence must be between 0.0 and 1.0')
        return v

    @validator('retrieval_time_ms', 'response_time_ms')
    def validate_time_metrics(cls, v):
        if v is not None and v <= 0:
            raise ValueError('time metrics must be positive')
        return v