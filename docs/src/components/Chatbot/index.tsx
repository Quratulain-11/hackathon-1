/**
 * Main Chatbot component that combines all chatbot functionality
 */
import React from 'react';
import ChatWidget from './ChatWidget';
import './Chatbot.css';

const Chatbot: React.FC = () => {
  return (
    <div className="docusaurus-chatbot">
      <ChatWidget />
    </div>
  );
};

export default Chatbot;