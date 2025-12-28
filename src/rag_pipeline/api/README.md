# RAG Chatbot API

A minimal FastAPI backend that connects retrieval (Spec 2) → Cohere Agent → API response, demonstrating end-to-end RAG flow.

## Features

- FastAPI-based REST API with automatic OpenAPI documentation
- RAG-based question answering using vector similarity search
- Integration with Cohere for response generation
- Structured responses with source attribution
- Performance timing metrics
- Confidence scoring

## Architecture

```
Query → Cohere Embedding → Qdrant Similarity Search → Retrieved Context → Cohere Response Generation → Answer
```

## Endpoints

### `POST /api/v1/chat`

Process a user query and return an AI-generated response based on retrieved document chunks.

**Request Body:**
```json
{
  "query": "Your question here",
  "top_k": 5,
  "temperature": 0.7,
  "max_tokens": 500
}
```

**Response:**
```json
{
  "answer": "Generated response text",
  "sources": [
    {
      "content": "Retrieved document content",
      "file_path": "path/to/source",
      "page_title": "Page title",
      "heading": "Section heading",
      "relevance_score": 0.85,
      "chunk_index": 1
    }
  ],
  "query": "Original query",
  "confidence": 0.85,
  "retrieval_time_ms": 245.3,
  "response_time_ms": 1200.7,
  "timestamp": "2025-12-21T20:00:00Z"
}
```

## Running the API

1. Make sure you have a `.env` file with your Cohere API key:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_url_here
   QDRANT_API_KEY=your_qdrant_api_key
   ```

2. Install dependencies:
   ```bash
   pip install -e .
   ```

3. Run the server:
   ```bash
   python -m src.rag_pipeline.api.server
   ```

4. Access the API at `http://localhost:8000`
5. API documentation available at `http://localhost:8000/docs`

## Testing

Run the validation test:
```bash
python test_api.py
```

## Limitations

See [limitations.md](../../../specs/3-agent-backend-api/limitations.md) for a complete list of current limitations and deferred features.