"""
Pydantic model for chat request in the RAG Chatbot API.
"""
from pydantic import BaseModel, validator
from typing import Optional


class ChatRequest(BaseModel):
    """
    Represents a user query sent to the chat endpoint.
    """
    query: str
    top_k: Optional[int] = 5
    temperature: Optional[float] = 0.7
    max_tokens: Optional[int] = 500

    @validator('query')
    def validate_query(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('query must not be empty')
        if len(v) > 1000:  # Query length limit to prevent abuse
            raise ValueError('query must not exceed 1000 characters')
        return v.strip()

    @validator('top_k')
    def validate_top_k(cls, v):
        if v is not None and (v < 1 or v > 20):
            raise ValueError('top_k must be between 1 and 20')
        return v

    @validator('temperature')
    def validate_temperature(cls, v):
        if v is not None and (v < 0.0 or v > 1.0):
            raise ValueError('temperature must be between 0.0 and 1.0')
        return v

    @validator('max_tokens')
    def validate_max_tokens(cls, v):
        if v is not None and (v < 1 or v > 2000):
            raise ValueError('max_tokens must be between 1 and 2000')
        return v