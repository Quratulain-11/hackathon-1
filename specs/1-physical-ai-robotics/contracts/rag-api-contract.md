# RAG API Contract: Physical AI & Humanoid Robotics

## Overview
This document defines the API contract for the Retrieval-Augmented Generation (RAG) system that will be embedded in the Physical AI & Humanoid Robotics book. The RAG system provides content-specific Q&A functionality without hallucination.

## Base URL
`https://nainee-chatbot.hf.space/api/v1` (deployed service)
`http://localhost:8000/api/v1` (for local development)
`https://[book-domain]/api/v1` (for production)

## Authentication
All API requests require an API key in the header:
```
Authorization: Bearer {api_key}
```

## Endpoints

### 1. Query Processing
**POST** `/rag/query`

#### Request
```json
{
  "query": "string (the user's question)",
  "context": {
    "module_id": "string (optional, current module)",
    "section_id": "string (optional, current section)",
    "conversation_history": [
      {
        "role": "user|assistant",
        "content": "string"
      }
    ]
  },
  "options": {
    "max_results": "integer (default: 5)",
    "min_relevance_score": "float (default: 0.7)",
    "include_citations": "boolean (default: true)"
  }
}
```

#### Response
```json
{
  "id": "string (unique query ID)",
  "query": "string (original query)",
  "response": "string (the generated answer)",
  "citations": [
    {
      "document_id": "string",
      "title": "string",
      "section": "string",
      "page_reference": "string",
      "relevance_score": "float",
      "text_snippet": "string"
    }
  ],
  "confidence_score": "float (0-1)",
  "hallucination_detected": "boolean",
  "generated_at": "timestamp",
  "sources": "string[] (list of source identifiers)"
}
```

#### Error Responses
- `400 Bad Request`: Invalid request format
- `401 Unauthorized`: Missing or invalid API key
- `422 Unprocessable Entity`: Query too short or contains invalid content
- `500 Internal Server Error`: Processing error

### 2. Document Management
**POST** `/rag/documents`

#### Request
```json
{
  "documents": [
    {
      "id": "string (unique document ID)",
      "title": "string",
      "content": "string (markdown content)",
      "module_id": "string",
      "section_id": "string",
      "page_reference": "string",
      "metadata": {
        "author": "string",
        "created_at": "timestamp",
        "updated_at": "timestamp",
        "tags": ["string"]
      }
    }
  ]
}
```

#### Response
```json
{
  "status": "success|error",
  "processed_count": "integer",
  "failed_count": "integer",
  "errors": [
    {
      "document_id": "string",
      "error": "string"
    }
  ]
}
```

### 3. Health Check
**GET** `/health`

#### Response
```json
{
  "status": "healthy",
  "timestamp": "timestamp",
  "services": {
    "vector_db": "healthy|unhealthy",
    "embedding_model": "healthy|unhealthy",
    "llm_service": "healthy|unhealthy"
  }
}
```

### 4. Chat History
**POST** `/rag/conversation`

#### Request
```json
{
  "session_id": "string (optional, auto-generated if not provided)",
  "query": "string",
  "response": "string",
  "user_id": "string (optional)"
}
```

#### Response
```json
{
  "session_id": "string",
  "query_id": "string",
  "timestamp": "timestamp"
}
```

**GET** `/rag/conversation/{session_id}`

#### Response
```json
{
  "session_id": "string",
  "history": [
    {
      "id": "string",
      "role": "user|assistant",
      "content": "string",
      "timestamp": "timestamp"
    }
  ],
  "created_at": "timestamp",
  "updated_at": "timestamp"
}
```

## Rate Limits
- Queries: 100 per minute per API key
- Document uploads: 10 per minute per API key

## Data Validation Rules
1. Query length: 5-1000 characters
2. Document content: 50-10000 characters per chunk
3. Maximum 100 citations returned per query
4. All responses must include source citations

## Response Guarantees
1. No hallucination: All responses based only on indexed content
2. Citations: Every claim attributed to source document
3. Relevance: Responses scored for relevance to query
4. Accuracy: 95%+ accuracy rate for content-based questions

## Error Handling
- Invalid queries return 422 with descriptive error message
- System errors return 500 with error ID for debugging
- Rate limit exceeded returns 429 with retry-after header
- Content not found returns 200 with empty citations array

## Versioning
- API version: v1 (current)
- Backward compatibility: Maintained for minor version changes
- Breaking changes: New major version with parallel support period