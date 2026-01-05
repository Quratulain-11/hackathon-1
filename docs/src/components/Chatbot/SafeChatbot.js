/**
 * Safe Chatbot Loader - Loads the chatbot without causing circular dependencies
 */
import React, { useEffect } from 'react';

const SafeChatbot = () => {
  useEffect(() => {
    // Create and load the chatbot dynamically after the DOM is ready
    const loadChatbot = async () => {
      if (typeof window !== 'undefined') {
        try {
          // Check if chatbot is already loaded to prevent duplicates
          if (document.getElementById('docusaurus-chatbot-container')) {
            return;
          }

          // Dynamically import the chatbot module and render it
          const { default: Chatbot } = await import('./index');

          // Create a container for the chatbot
          const container = document.createElement('div');
          container.id = 'docusaurus-chatbot-container';
          container.className = 'docusaurus-chatbot';
          document.body.appendChild(container);

          // Render the chatbot using React DOM
          const { createRoot } = await import('react-dom/client');
          const root = createRoot(container);
          root.render(<Chatbot />);
        } catch (error) {
          console.error('Failed to load chatbot:', error);
        }
      }
    };

    // Wait for the DOM to be ready before loading the chatbot
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', loadChatbot);
    } else {
      loadChatbot();
    }

    // Cleanup function
    return () => {
      const existingContainer = document.getElementById('docusaurus-chatbot-container');
      if (existingContainer) {
        existingContainer.remove();
      }
    };
  }, []);

  // This component doesn't render anything itself
  // The chatbot is rendered directly to the DOM
  return null;
};

export default SafeChatbot;