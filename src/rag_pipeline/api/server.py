"""
Development server for the RAG Chatbot API.
"""
import uvicorn
import sys
from .main import app


def run_server():
    """Run the FastAPI server."""
    print("Starting RAG Chatbot API server...")
    print("Access the API documentation at: http://localhost:8000/docs")

    uvicorn.run(
        "src.rag_pipeline.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )


if __name__ == "__main__":
    run_server()