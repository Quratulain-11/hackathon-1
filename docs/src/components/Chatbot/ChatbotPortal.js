/**
 * Chatbot Portal - A component that safely renders the chatbot without circular dependencies
 * Uses React portals to render outside the normal component hierarchy
 */
import React, { useEffect, useState } from 'react';
import { createPortal } from 'react-dom';

// A self-contained chatbot loader that avoids circular dependencies
const ChatbotPortal = () => {
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  // Create portal container element
  useEffect(() => {
    if (!isClient) return;

    const chatbotContainer = document.createElement('div');
    chatbotContainer.id = 'docusaurus-chatbot-portal';
    document.body.appendChild(chatbotContainer);

    // Clean up on unmount
    return () => {
      const existingContainer = document.getElementById('docusaurus-chatbot-portal');
      if (existingContainer && existingContainer.parentNode) {
        existingContainer.parentNode.removeChild(existingContainer);
      }
    };
  }, [isClient]);

  if (!isClient) {
    return null;
  }

  // Dynamically import and render the chatbot component
  const [ChatbotComponent, setChatbotComponent] = useState(null);

  useEffect(() => {
    const loadChatbot = async () => {
      try {
        const { default: Chatbot } = await import('./index');
        setChatbotComponent(Chatbot);
      } catch (error) {
        console.error('Failed to load chatbot:', error);
      }
    };

    loadChatbot();
  }, []);

  const portalContainer = document.getElementById('docusaurus-chatbot-portal');

  if (!ChatbotComponent || !portalContainer) {
    return null;
  }

  return createPortal(<ChatbotComponent />, portalContainer);
};

export default ChatbotPortal;