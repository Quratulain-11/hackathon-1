/**
 * Chatbot Portal - A component that safely renders the chatbot without circular dependencies
 * Uses React portals to render outside the normal component hierarchy
 */
import React, { useEffect, useState } from 'react';
import { createPortal } from 'react-dom';

// A self-contained chatbot loader that avoids circular dependencies
const ChatbotPortal = () => {
  const [chatbotElement, setChatbotElement] = useState(null);
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);

    // Create a container element for the chatbot
    const chatbotContainer = document.createElement('div');
    chatbotContainer.id = 'docusaurus-chatbot-portal';
    document.body.appendChild(chatbotContainer);

    setChatbotElement(chatbotContainer);

    // Clean up on unmount
    return () => {
      if (chatbotContainer && chatbotContainer.parentNode) {
        chatbotContainer.parentNode.removeChild(chatbotContainer);
      }
    };
  }, []);

  // Dynamically load and render the chatbot component
  useEffect(() => {
    if (!chatbotElement || !isClient) return;

    const loadAndRenderChatbot = async () => {
      try {
        // Dynamically import the chatbot component
        const { default: Chatbot } = await import('./index');

        // Create a React component that renders the chatbot
        const ChatbotComponent = () => <Chatbot />;

        // Render the chatbot into the portal container
        // We'll use a simple approach by rendering it directly
        if (chatbotElement) {
          // We'll handle the rendering differently to avoid direct circular imports
          // Instead, we'll use dynamic import and render to the DOM element
          const renderChatbot = async () => {
            try {
              const { createRoot } = await import('react-dom/client');
              const chatbotModule = await import('./index');
              if (chatbotModule.default && chatbotElement) {
                const root = createRoot(chatbotElement);
                root.render(<chatbotModule.default />);
              }
            } catch (error) {
              console.error('Failed to render chatbot:', error);
            }
          };

          renderChatbot();
        }
      } catch (error) {
        console.error('Failed to load chatbot:', error);
      }
    };

    loadAndRenderChatbot();
  }, [chatbotElement, isClient]);

  // Return null since the chatbot is rendered via portal
  return null;
};

export default ChatbotPortal;