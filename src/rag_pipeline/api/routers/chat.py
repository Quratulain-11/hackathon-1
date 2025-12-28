"""
Chat router for the RAG Chatbot API.
"""
from fastapi import APIRouter, HTTPException, Depends
from typing import List
import logging
from datetime import datetime

from ..models.chat_request import ChatRequest
from ..models.chat_response import ChatResponse
from ..models.error_response import ErrorResponse
from ..services.agent_service import AgentService
from ...config import Config

logger = logging.getLogger(__name__)

router = APIRouter()

def get_config():
    """Get configuration instance."""
    return Config()

# We don't need the dependency function anymore since we access the service directly in the endpoint


@router.post("/chat", response_model=ChatResponse, summary="Chat endpoint")
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that accepts a user query and returns an AI-generated response
    based on retrieved document chunks.
    """
    try:
        # Get agent service from app state
        agent_service = request.app.state.agent_service

        # Process the query through the agent service
        result = await agent_service.process_query(
            query=request.query,
            top_k=request.top_k,
            temperature=request.temperature,
            max_tokens=request.max_tokens
        )

        # Create and return the response
        response = ChatResponse(
            answer=result["answer"],
            sources=result["sources"],
            query=result["query"],
            confidence=result["confidence"],
            retrieval_time_ms=result["retrieval_time_ms"],
            response_time_ms=result["response_time_ms"],
            timestamp=result["timestamp"]
        )

        logger.info(f"Chat request processed successfully. Query: {request.query[:50]}...")

        return response

    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        error_response = ErrorResponse(
            error=str(e),
            error_code="CHAT_PROCESSING_ERROR",
            timestamp=datetime.now()
        )
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/health", summary="Health check")
async def health():
    """Health check endpoint for the chat router."""
    return {"status": "healthy", "timestamp": datetime.now().isoformat()}