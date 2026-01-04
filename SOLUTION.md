# Solution: Documentation Assistant Query Embedding Issue

## Problem
The documentation assistant is showing "Failed to generate query embedding" error when trying to answer questions.

## Root Causes Identified
1. **API Rate Limiting**: The Cohere API is returning 429 (Too Many Requests) errors
2. **Missing Document Index**: The Qdrant database likely doesn't have any documents indexed yet
3. **Error Handling**: The Cohere client had incorrect error handling for newer Cohere library versions and internal library errors

## Solution Steps

### 1. Fixed Error Handling
- Updated Cohere client to properly handle internal library errors and dynamic import issues
- Implemented comprehensive exception handling for various error types and scenarios
- Added specific handling for the "CohereError found in _dynamic_imports" error

### 2. Reduced API Request Frequency
- Added configuration to reduce concurrent requests: `MAX_CONCURRENT_REQUESTS=2`
- Increased request timeout: `REQUEST_TIMEOUT=60`

### 3. Populate Document Database
To fix the issue, you need to index your documentation first:

```bash
cd backend
python -m src.rag_pipeline.cli.main process --docs-path ../docs/docs
```

### 4. Start the Backend Server
```bash
cd backend
python -m uvicorn src.rag_pipeline.api.main:app --host 0.0.0.0 --port 8000
```

### 5. Alternative: Use Local Qdrant (Recommended)
If you're having issues with the cloud Qdrant, you can use a local instance:

1. Start local Qdrant:
```bash
docker run -d --name qdrant-container -p 6333:6333 -p 6334:6334 \
    -v $(pwd)/qdrant_data:/qdrant/storage:z qdrant/qdrant
```

2. Update your .env file:
```
# Comment out cloud Qdrant settings
# QDRANT_URL="https://..."
# QDRANT_API_KEY="..."

# Use local Qdrant
QDRANT_HOST=localhost
QDRANT_PORT=6333
```

## Verification
After indexing documents and starting the server, test the API:

```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is this documentation about?",
    "top_k": 3
  }'
```

## Additional Notes
- The Cohere API key in the example is a placeholder - replace with your actual API key
- The OpenRouter API key is needed for response generation
- Rate limiting is common with free API tiers - consider upgrading for higher limits
- Documents with fewer than 50 tokens are skipped during processing
- The error handling now properly addresses internal Cohere library issues