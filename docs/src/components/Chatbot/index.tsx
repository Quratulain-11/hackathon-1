/**
 * Main Chatbot component that combines all chatbot functionality
 */
import React from 'react';
import ChatWidget from './ChatWidget';
import './Chatbot.css';

const Chatbot: React.FC = () => {
  return (
    <div className="docusaurus-chatbot" style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 1000 }}>
      <ChatWidget />
    </div>
  );
};

export default Chatbot;