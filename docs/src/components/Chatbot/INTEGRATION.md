# Frontend-Backend Integration Guide

This document provides instructions for integrating the RAG Chatbot frontend with the FastAPI backend.

## Overview

The RAG Chatbot frontend is a React component that integrates with a Docusaurus documentation site and communicates with a FastAPI RAG backend via HTTP. The chatbot appears as a floating widget on every documentation page.

## Architecture

- **Frontend**: React/TypeScript components integrated into Docusaurus
- **Backend**: FastAPI service with `/api/v1/chat` endpoint
- **Communication**: JSON over HTTP
- **State Management**: React hooks
- **Styling**: CSS modules

## Backend Requirements

The frontend expects the following from the backend:

1. A `/api/v1/chat` endpoint that accepts POST requests
2. Request format:
   ```json
   {
     "query": "user question",
     "top_k": 5,
     "temperature": 0.7,
     "max_tokens": 500
   }
   ```
3. Response format:
   ```json
   {
     "response": "answer to the question",
     "sources": ["array", "of", "source", "documents"]
   }
   ```

## Configuration

The backend URL is configurable via environment variables:

```bash
REACT_APP_BACKEND_URL=https://nainee-chatbot.hf.space  # Default
```

## Running Together

### Prerequisites

- Node.js v14+ for the frontend
- Python v3.8+ for the backend
- FastAPI RAG backend service

### Steps

1. **Start the Backend**
   ```bash
   # In your backend directory
   uvicorn main:app --reload --port 8000
   ```

2. **Start the Frontend**
   ```bash
   # In your Docusaurus project directory
   npm start
   ```

3. **Access the Documentation Site**
   - Frontend will be available at `http://localhost:3000`
   - Chatbot widget will appear on all documentation pages

## UI Testing Without Backend

The chatbot UI will render and be visible on all pages even when the backend is not running. When the backend is unavailable:

- The chat button will still appear on every page
- The chat panel will open when clicked
- Attempting to send messages will show a "Backend not connected" message
- The UI remains fully functional for visual testing

## Troubleshooting

### Common Issues

- **CORS Errors**: Ensure your FastAPI backend allows requests from your frontend origin
- **Connection Refused**: Verify the backend is running and accessible at the configured URL
- **API Key Errors**: Check that your backend has proper API keys configured
- **UI Not Appearing**: Verify that the Layout override is properly implemented

### Environment Variables

- `REACT_APP_BACKEND_URL`: Backend URL (default: `https://nainee-chatbot.hf.space`)

## API Error Handling

The frontend handles various backend errors gracefully:

- Network errors: Shows user-friendly message
- API key errors: Displays appropriate error without crashing
- Timeout errors: Notifies user of slow response
- 429 (Rate limit): Implements retry mechanism
- 5xx errors: Provides fallback messaging

## Testing

To test the integration:

1. Ensure both backend and frontend are running
2. Navigate to any documentation page
3. Click the chat widget button
4. Enter a question and submit
5. Verify the response appears in the chat

## Security Considerations

- API keys are managed by the backend
- No sensitive data is stored in the frontend
- All communication is over HTTP (consider HTTPS for production)