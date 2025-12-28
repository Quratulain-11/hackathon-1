# Running the RAG Chatbot

## Current Status

The RAG (Retrieval-Augmented Generation) Chatbot is now successfully running! Here's what has been accomplished:

✅ **API Server**: Running at `http://localhost:8000`
✅ **API Documentation**: Available at `http://localhost:8000/docs`
✅ **Qdrant Vector Database**: Running in local mode with file storage
✅ **Documentation**: Docusaurus documentation files identified and ready for processing

## Current Configuration

- **Backend**: FastAPI server with Cohere integration
- **Vector Database**: Qdrant in local file storage mode (data stored in `./qdrant_data/`)
- **Documentation**: Located in `./docs/docs/` directory

## Next Steps for Full Functionality

To make the chatbot fully functional, you need to:

1. **Get a Cohere API Key**:
   - Visit [https://dashboard.cohere.com/](https://dashboard.cohere.com/)
   - Sign up for an account
   - Navigate to "API Keys" section
   - Create a new API key

2. **Update the Environment File**:
   - Replace `your-cohere-api-key-here` in `.env` with your actual API key
   - Example:
     ```
     COHERE_API_KEY=your-actual-cohere-api-key-here
     ```

3. **Process Documentation** (after getting API key):
   ```bash
   python -m src.rag_pipeline.cli.main process --docs-path docs/docs --collection-name docusaurus-content
   ```

## Testing the Chatbot

Once you have a Cohere API key and have processed the documentation:

1. Send a test query using curl:
   ```bash
   curl -X POST http://localhost:8000/api/v1/chat \
     -H "Content-Type: application/json" \
     -d '{
       "query": "What is the ROS 2 nervous system architecture?",
       "top_k": 5,
       "temperature": 0.7
     }'
   ```

2. Or use the API documentation interface at `http://localhost:8000/docs` to test interactively.

## Troubleshooting

- If the server is not running, restart it with: `python -m src.rag_pipeline.api.server`
- If you get Cohere-related errors, make sure your API key is correctly configured
- Qdrant data is stored in the `./qdrant_data/` directory

## Architecture Overview

The chatbot follows this flow:
1. User sends a query to the API
2. Query is embedded using Cohere
3. Vector similarity search is performed in Qdrant to find relevant documents
4. Retrieved context is sent to Cohere for response generation
5. Response includes answer, sources, confidence score, and timing metrics