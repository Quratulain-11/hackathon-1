# Quickstart: RAG Chatbot Agent-Based Backend API

## Overview

This guide will help you set up and run the RAG Chatbot backend API. The API provides a chat endpoint that accepts user queries, retrieves relevant document chunks, and generates AI-powered responses based on the retrieved content.

## Prerequisites

Before starting, ensure you have:

1. **Qdrant Database**: Running with embedded book content (see `1-rag-book-embeddings` feature)
2. **Cohere API Key**: Valid API key in your `.env` file
3. **Embedded Content**: Document chunks already stored in Qdrant collection
4. **Python 3.11+**: Required for the project dependencies

## Setup

### 1. Environment Configuration

Make sure your `.env` file contains:

```bash
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here  # or use local instance
QDRANT_API_KEY=your_qdrant_api_key  # if using cloud instance
QDRANT_COLLECTION_NAME=docusaurus-content  # or your collection name
```

### 2. Install Dependencies

If you haven't already:

```bash
pip install -e .
```

### 3. Install FastAPI and Uvicorn

```bash
pip install fastapi uvicorn
```

## Running the API

### 1. Start the API Server

```bash
uvicorn src.rag_pipeline.api.main:app --reload --port 8000
```

The API will be available at `https://nainee-chatbot.hf.space`.

### 2. Access API Documentation

FastAPI automatically provides interactive API documentation:

- Swagger UI: `https://nainee-chatbot.hf.space/docs`
- ReDoc: `https://nainee-chatbot.hf.space/redoc`

## Using the Chat Endpoint

### 1. Send a Query via cURL

```bash
curl -X POST https://nainee-chatbot.hf.space/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the ROS 2 nervous system architecture?",
    "top_k": 5,
    "temperature": 0.7
  }'
```

### 2. Send a Query via Python

```python
import requests

response = requests.post(
    "https://nainee-chatbot.hf.space/chat",
    json={
        "query": "What is the ROS 2 nervous system architecture?",
        "top_k": 5,
        "temperature": 0.7
    }
)

print(response.json())
```

## API Response Format

The API returns responses in the following format:

```json
{
  "answer": "The ROS 2 nervous system architecture consists of...",
  "sources": [
    {
      "content": "ROS 2 provides a middleware layer that enables communication...",
      "file_path": "docs/docs/module-1-ros-nervous-system/index.md",
      "page_title": "Module 1: ROS 2 Nervous System",
      "heading": "Architecture Overview",
      "relevance_score": 0.92,
      "chunk_index": 2
    }
  ],
  "query": "What is the ROS 2 nervous system architecture?",
  "confidence": 0.85,
  "retrieval_time_ms": 245.3,
  "response_time_ms": 1200.7,
  "timestamp": "2025-12-21T20:00:00Z"
}
```

## Configuration Options

### Query Parameters

- `query`: (required) The user's query text
- `top_k`: (optional) Number of document chunks to retrieve (default: 5, range: 1-20)
- `temperature`: (optional) Response randomness (default: 0.7, range: 0.0-1.0)
- `max_tokens`: (optional) Maximum response length (default: 500, range: 1-2000)

## Troubleshooting

### Common Issues

1. **Connection Errors**: Verify Qdrant URL and credentials in `.env`
2. **API Errors**: Check Cohere API key validity
3. **No Results**: Ensure document chunks are properly embedded in Qdrant
4. **Slow Responses**: Consider reducing top_k parameter or optimizing query

### Verification Steps

1. Check API health: `curl https://nainee-chatbot.hf.space/health`
2. Verify API documentation loads at `https://nainee-chatbot.hf.space/docs`
3. Test with a simple query to ensure basic functionality

## Next Steps

1. Integrate the API into your frontend application
2. Implement error handling for production use
3. Add authentication and rate limiting as needed
4. Monitor API performance and usage metrics