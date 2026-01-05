/**
 * ChatWidget component - Main container for the chatbot UI with state management
 */
import React, { useState, useEffect } from 'react';
import ChatButton from './ChatButton';
import ChatPanel from './ChatPanel';
import { sendMessage } from './api/client';
import { ChatMessage } from './types';
import { formatErrorMessage } from './utils';
import { BACKEND_CONFIG } from './config';

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Add initial welcome message
  useEffect(() => {
    if (isOpen && messages.length === 0) {
      setMessages([
        {
          id: 'welcome',
          role: 'assistant',
          content: 'Hello! I\'m your documentation assistant. Ask me anything about this book.',
          timestamp: new Date(),
        },
      ]);
    }
  }, [isOpen, messages.length]);


  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    // Input validation
    const trimmedInput = inputValue.trim();

    // Check if input is empty
    if (!trimmedInput) {
      setError('Please enter a question before submitting.');
      return;
    }

    // Check if input is too long (e.g., more than 2000 characters)
    if (trimmedInput.length > 2000) {
      setError('Query is too long. Please keep your question under 2000 characters.');
      return;
    }

    if (isLoading) return;

    try {
      // Add user message to the chat
      const userMessage: ChatMessage = {
        id: Date.now().toString(),
        role: 'user',
        content: trimmedInput,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, userMessage]);
      setInputValue('');
      setIsLoading(true);
      setError(null);

      // Call the API to get response
      const response = await sendMessage(trimmedInput);

      // Add assistant response to the chat
      const assistantMessage: ChatMessage = {
        id: `response-${Date.now()}`,
        role: 'assistant',
        content: response.answer || 'Sorry, I couldn\'t process that request.',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Error sending message:', err);
      const errorMessage = formatErrorMessage(err);

      // Add error message to chat, but make it user-friendly
      const errorChatMessage: ChatMessage = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: errorMessage,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorChatMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Add keyboard shortcut to open/close chat
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Ctrl+K or Cmd+K to toggle chat
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        setIsOpen(prev => !prev);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, []);

  return (
    <div className="chat-widget">
      <ChatButton isOpen={isOpen} onClick={() => setIsOpen(!isOpen)} />
      {isOpen && (
        <ChatPanel
          messages={messages}
          inputValue={inputValue}
          setInputValue={setInputValue}
          onSubmit={handleSubmit}
          isLoading={isLoading}
          error={error}
        />
      )}
    </div>
  );
};

export default ChatWidget;