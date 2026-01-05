# RAG Chatbot Frontend-Backend Integration

This documentation describes how to run the Docusaurus frontend with the FastAPI RAG backend together.

## Prerequisites

- Node.js (v14 or higher)
- Python (v3.8 or higher)
- The FastAPI RAG backend service (optional for UI testing)

## Running the Backend

1. Navigate to your FastAPI RAG backend directory
2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Start the backend server:
   ```bash
   uvicorn main:app --reload --port 8000
   ```
   The backend will be available at `https://nainee-chatbot.hf.space`

## Running the Frontend

1. Install dependencies:
   ```bash
   npm install
   ```
2. Start the Docusaurus development server:
   ```bash
   npm start
   ```
3. The frontend will be available at `http://localhost:3000`

## Configuration

The frontend is configured to connect to the backend at `https://nainee-chatbot.hf.space` by default. To change this, you can set the `REACT_APP_BACKEND_URL` environment variable:

```bash
export REACT_APP_BACKEND_URL=https://nainee-chatbot.hf.space
```

## Environment Variables

- `REACT_APP_BACKEND_URL`: The URL of the FastAPI backend (default: `https://nainee-chatbot.hf.space`)

## Usage

1. Once both services are running, visit the Docusaurus site in your browser
2. You'll see a floating chat button in the bottom-right corner of every page
3. Click the button to open the chat interface
4. Type your question about the documentation and press Enter or click the send button
5. The response from the RAG backend will appear in the chat

## UI Testing Without Backend

The chatbot UI will render and be visible on all pages even when the backend is not running. When the backend is unavailable:

- The chat button will still appear on every page
- The chat panel will open when clicked
- Attempting to send messages will show a "Backend not connected" message
- The UI remains fully functional for visual testing

## Troubleshooting

### Common Issues

- **CORS errors**: Make sure your FastAPI backend allows requests from `http://localhost:3000`
- **Connection errors**: Verify that the backend server is running and accessible at the configured URL
- **API key errors**: Check that your backend is properly configured with required API keys

### Backend Configuration

If your backend requires specific API keys or configuration, make sure those are properly set in the backend environment before starting the service.