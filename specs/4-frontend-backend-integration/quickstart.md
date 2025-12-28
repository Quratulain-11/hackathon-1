# Quickstart: RAG Chatbot Frontend-Backend Integration

## Overview

This guide will help you integrate the RAG chatbot into your Docusaurus documentation site. The integration adds a chatbot UI component that communicates with your existing FastAPI RAG backend.

## Prerequisites

Before starting, ensure you have:

1. **Running FastAPI Backend**: The backend with the `/chat` endpoint must be accessible
2. **Docusaurus Site**: An existing Docusaurus documentation site
3. **Node.js**: Required for Docusaurus development (version 18+ recommended)
4. **Yarn or npm**: Package manager for installing dependencies

## Setup

### 1. Backend Configuration

Make sure your FastAPI backend is running and accessible. By default, the frontend expects the backend at `http://localhost:8000`. You can configure this with environment variables:

```bash
# .env file in your Docusaurus root
REACT_APP_BACKEND_URL=http://localhost:8000
# or for production
REACT_APP_BACKEND_URL=https://your-backend-domain.com
```

### 2. Install Dependencies

Navigate to your Docusaurus directory and install required dependencies:

```bash
npm install
# or if using yarn
yarn install
```

## Implementation

### 1. Create Chatbot Component

Create the chatbot component in `docs/src/components/Chatbot/Chatbot.tsx`:

```typescript
import React, { useState } from 'react';
import './Chatbot.css';

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
}

const Chatbot: React.FC = () => {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: ChatMessage = { role: 'user', content: inputValue };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${BACKEND_URL}/api/v1/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          top_k: 5,
          temperature: 0.7,
          max_tokens: 500
        }),
      });

      if (!response.ok) {
        throw new Error(`Backend error: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant response
      const assistantMessage: ChatMessage = {
        role: 'assistant',
        content: data.answer
      };
      setMessages(prev => [...prev, assistantMessage]);
      setInputValue('');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An unknown error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>RAG Chat Assistant</h3>
      </div>

      <div className="chatbot-messages">
        {messages.map((msg, index) => (
          <div key={index} className={`message ${msg.role}`}>
            <div className="message-content">{msg.content}</div>
          </div>
        ))}

        {isLoading && (
          <div className="message assistant">
            <div className="message-content">Thinking...</div>
          </div>
        )}

        {error && (
          <div className="message error">
            <div className="message-content">Error: {error}</div>
          </div>
        )}
      </div>

      <form onSubmit={handleSubmit} className="chatbot-input-form">
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask about the documentation..."
          disabled={isLoading}
          className="chatbot-input"
        />
        <button type="submit" disabled={isLoading || !inputValue.trim()}>
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default Chatbot;
```

### 2. Add CSS Styling

Create `docs/src/components/Chatbot/Chatbot.css`:

```css
.chatbot-container {
  border: 1px solid #ddd;
  border-radius: 8px;
  padding: 1rem;
  margin: 1rem 0;
  box-shadow: 0 2px 10px rgba(0,0,0,0.1);
  min-height: 400px;
  display: flex;
  flex-direction: column;
}

.chatbot-header h3 {
  margin: 0 0 1rem 0;
  color: #2563eb;
}

.chatbot-messages {
  flex-grow: 1;
  overflow-y: auto;
  max-height: 300px;
  margin-bottom: 1rem;
}

.message {
  margin-bottom: 0.5rem;
  padding: 0.5rem;
  border-radius: 4px;
}

.message.user {
  background-color: #dbeafe;
  margin-left: 20%;
  text-align: right;
}

.message.assistant {
  background-color: #f3f4f6;
  margin-right: 20%;
}

.message.error {
  background-color: #fee2e2;
  color: #dc2626;
}

.message-content {
  white-space: pre-wrap;
}

.chatbot-input-form {
  display: flex;
  gap: 0.5rem;
}

.chatbot-input {
  flex-grow: 1;
  padding: 0.5rem;
  border: 1px solid #d1d5db;
  border-radius: 4px;
}

.chatbot-input:focus {
  outline: none;
  border-color: #2563eb;
  box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.1);
}

.chatbot-input:disabled {
  background-color: #f3f4f6;
  cursor: not-allowed;
}

.chatbot-input-form button {
  padding: 0.5rem 1rem;
  background-color: #2563eb;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
}

.chatbot-input-form button:disabled {
  background-color: #9ca3af;
  cursor: not-allowed;
}
```

### 3. Integrate with Docusaurus Theme

Create a theme override to inject the chatbot globally. Create `src/theme/Layout/index.js`:

```javascript
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot/Chatbot';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {/* Add the chatbot component */}
        <div style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          zIndex: 1000,
          width: '400px',
          maxHeight: '600px'
        }}>
          <Chatbot />
        </div>
        {props.children}
      </OriginalLayout>
    </>
  );
}
```

Alternatively, if you prefer to add it inline on each page, you can add it to the sidebar or footer instead.

### 4. Update Docusaurus Configuration

Add the necessary configuration to `docusaurus.config.js` if needed:

```javascript
// docusaurus.config.js
module.exports = {
  // ... other config
  themes: [
    // ... other themes
  ],
  plugins: [
    // ... other plugins
  ],
  // Add any additional configuration for the chatbot if needed
};
```

## Running the Integration

### 1. Start Your Backend

Make sure your FastAPI backend is running:

```bash
# In your backend directory
python -m src.rag_pipeline.api.server
```

### 2. Start Your Docusaurus Site

```bash
# In your Docusaurus directory
npm run start
# or
yarn start
```

The chatbot should now be visible on all documentation pages.

## Testing the Integration

### 1. Basic Functionality Test

1. Navigate to any documentation page
2. Type a query in the chatbot input field
3. Verify the query is sent to the backend
4. Verify the response is displayed in the UI

### 2. Error Handling Test

1. Temporarily stop the backend server
2. Try submitting a query
3. Verify an appropriate error message is displayed
4. Restart the backend and verify normal functionality resumes

### 3. Configuration Test

1. Change the backend URL in your environment variables
2. Restart the Docusaurus development server
3. Verify the chatbot connects to the new backend URL

## Troubleshooting

### Common Issues

1. **CORS Errors**: Make sure your FastAPI backend has CORS configured to allow requests from your Docusaurus site
2. **Network Errors**: Verify the backend URL is accessible from the browser
3. **Environment Variables**: Ensure REACT_APP_BACKEND_URL is properly set and prefixed with REACT_APP_

### Verification Steps

1. Check browser console for any JavaScript errors
2. Verify network requests are being made to the correct backend URL
3. Test with simple queries first to ensure basic functionality works
4. Check that environment variables are properly loaded in development

## Next Steps

1. Customize the chatbot UI to match your site's design
2. Add more sophisticated error handling and user feedback
3. Implement additional features like message history or conversation persistence
4. Test the integration across different browsers and devices