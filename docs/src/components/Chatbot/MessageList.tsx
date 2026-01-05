/**
 * MessageList component - Renders the list of chat messages
 */
import React from 'react';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

interface MessageListProps {
  messages: Message[];
}

const MessageList: React.FC<MessageListProps> = ({ messages }) => {
  return (
    <div className="chat-messages">
      {messages.map((message) => (
        <div
          key={message.id}
          className={`message message--${message.role}`}
          aria-live={message.role === 'assistant' ? 'polite' : 'off'}
        >
          <div className="message-content">
            {message.content}
          </div>
          <div className="message-timestamp">
            {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
          </div>
        </div>
      ))}
    </div>
  );
};

export default MessageList;