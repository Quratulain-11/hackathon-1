"""
Pydantic model for error responses in the RAG Chatbot API.
"""
from pydantic import BaseModel, validator
from typing import Optional, Dict, Any
from datetime import datetime


class ErrorResponse(BaseModel):
    """
    Represents an error response when the chat endpoint encounters an issue.
    """
    error: str
    error_code: str
    details: Optional[Dict[str, Any]] = None
    timestamp: datetime

    @validator('error')
    def validate_error(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('error must not be empty')
        return v

    @validator('error_code')
    def validate_error_code(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('error_code must not be empty')
        return v