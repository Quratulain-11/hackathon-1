/**
 * Safe Chatbot Loader - Fully interactive chatbot with RAG capabilities
 */
import React, { useState, useEffect, useRef } from 'react';
import { sendMessage } from './api/client';
import './Chatbot.css';

const SafeChatbot = () => {
  const [isVisible, setIsVisible] = useState(false);
  const [isMinimized, setIsMinimized] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);

  // Toggle chatbot visibility
  const toggleChatbot = () => {
    console.log('Toggle chatbot clicked'); // Debug log
    setIsVisible(prevIsVisible => {
      const newIsVisible = !prevIsVisible;
      console.log(`Setting isVisible to: ${newIsVisible}`); // Debug log

      if (!prevIsVisible) {
        // We're opening the chatbot
        setIsMinimized(false);

        // Use callback form to ensure we have the current messages state
        setMessages(prevMessages => {
          if (prevMessages.length === 0) {
            console.log('Adding welcome message'); // Debug log
            return [
              {
                id: 'welcome',
                role: 'assistant',
                content: 'Hello! I\'m your documentation assistant. Ask me anything about the Physical AI & Humanoid Robotics book.',
                timestamp: new Date(),
              },
            ];
          }
          return prevMessages;
        });
      }

      return newIsVisible;
    });
  };

  // Close chatbot
  const closeChatbot = () => {
    console.log('Close chatbot clicked');
    setIsVisible(false);
    setIsMinimized(false);
    setMessages([]); // This is fine since we're resetting to an empty array
  };

  // Minimize/maximize chatbot
  const toggleMinimize = () => {
    console.log('Toggle minimize clicked');
    setIsMinimized(prev => !prev);
  };

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Handle keyboard shortcut (Ctrl/Cmd + K to toggle)
  useEffect(() => {
    const handleKeyDown = (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        console.log('Keyboard shortcut triggered - toggling chatbot');
        toggleChatbot();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [toggleChatbot]);

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) {
      return;
    }

    const userMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue.trim(),
      timestamp: new Date(),
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Call the API to get response
      const response = await sendMessage(inputValue.trim());

      // Add assistant response to the chat
      const assistantMessage = {
        id: `response-${Date.now()}`,
        role: 'assistant',
        content: response.answer,
        timestamp: new Date(),
        sources: response.sources || [],
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Error sending message:', err);

      // Add error message to chat
      const errorMessage = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: `Sorry, I couldn't process that request. ${err.message || 'Please try again.'}`,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* Chatbot toggle button */}
      {!isVisible && (
        <button
          className="chatbot-toggle-button"
          onClick={(e) => {
            console.log('Button clicked, calling toggleChatbot');
            e.stopPropagation();
            e.preventDefault();
            toggleChatbot();
          }}
          aria-label="Open chatbot"
          title="Open AI Assistant"
        >
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H7L3 21V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </button>
      )}

      {/* Chatbot container */}
      {isVisible && (
        <div className={`chatbot-container ${isMinimized ? 'minimized' : 'expanded'}`}>
          {/* Chatbot header */}
          <div className="chatbot-header">
            <div className="chatbot-title">AI Assistant</div>
            <div className="chatbot-controls">
              <button
                className="chatbot-minimize"
                onClick={toggleMinimize}
                aria-label={isMinimized ? "Maximize" : "Minimize"}
              >
                {isMinimized ? '+' : '−'}
              </button>
              <button
                className="chatbot-close"
                onClick={closeChatbot}
                aria-label="Close"
              >
                ×
              </button>
            </div>
          </div>

          {/* Chatbot content */}
          {!isMinimized && (
            <div className="chatbot-content">
              {/* Messages container */}
              <div className="chat-messages">
                {messages.map((message) => (
                  <div
                    key={message.id}
                    className={`message ${message.role === 'user' ? 'message--user' : 'message--assistant'}`}
                  >
                    <div className="message-content">
                      {message.content}
                      {message.sources && message.sources.length > 0 && (
                        <div className="message-sources">
                          <details>
                            <summary>Sources:</summary>
                            <ul>
                              {message.sources.slice(0, 3).map((source, index) => (
                                <li key={index}>
                                  {source.title || source.content?.substring(0, 50) + '...'}
                                </li>
                              ))}
                            </ul>
                          </details>
                        </div>
                      )}
                    </div>
                    <div className="message-timestamp">
                      {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                    </div>
                  </div>
                ))}
                {isLoading && (
                  <div className="message message--assistant">
                    <div className="message-content">
                      <div className="loading-spinner">Thinking...</div>
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>

              {/* Input form */}
              <form onSubmit={handleSubmit} className="chat-input-form">
                <input
                  type="text"
                  value={inputValue}
                  onChange={(e) => setInputValue(e.target.value)}
                  placeholder="Ask about the documentation..."
                  className="chat-input"
                  disabled={isLoading}
                  aria-label="Type your message"
                />
                <button
                  type="submit"
                  disabled={!inputValue.trim() || isLoading}
                  className="chat-submit-button"
                  aria-label="Send message"
                >
                  <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                  </svg>
                </button>
              </form>

              {/* Error message */}
              {error && (
                <div className="chat-error">
                  {error}
                </div>
              )}
            </div>
          )}
        </div>
      )}
    </>
  );
};

export default SafeChatbot;