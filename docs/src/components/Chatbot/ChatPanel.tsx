/**
 * ChatPanel component - Main chat panel with header, messages, and input
 */
import React from 'react';
import MessageList from './MessageList';

interface ChatPanelProps {
  messages: Array<{
    id: string;
    role: 'user' | 'assistant';
    content: string;
    timestamp: Date;
  }>;
  inputValue: string;
  setInputValue: (value: string) => void;
  onSubmit: (e: React.FormEvent) => void;
  isLoading: boolean;
  error: string | null;
}

const ChatPanel: React.FC<ChatPanelProps> = ({
  messages,
  inputValue,
  setInputValue,
  onSubmit,
  isLoading,
  error,
}) => {
  return (
    <div className="chat-panel">
      <div className="chat-header">
        <h3>Documentation Assistant</h3>
      </div>

      <MessageList messages={messages} />

      {error && (
        <div className="chat-error">
          <p>{error}</p>
        </div>
      )}

      <form onSubmit={onSubmit} className="chat-input-form">
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about the documentation..."
          disabled={isLoading}
          className="chat-input"
        />
        <button
          type="submit"
          disabled={isLoading || !inputValue.trim()}
          className="chat-submit-button"
        >
          {isLoading ? (
            <span className="loading-spinner">Sending...</span>
          ) : (
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
            </svg>
          )}
        </button>
      </form>
    </div>
  );
};

export default ChatPanel;