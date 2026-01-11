/**
 * Backend configuration utility for accessing config through Docusaurus context
 */

// Function to get the backend URL from window or use default
export const getBackendUrl = () => {
  // Try to get from window object which can be set by Docusaurus
  if (typeof window !== 'undefined' && window.__BACKEND_CONFIG__) {
    return window.__BACKEND_CONFIG__.baseUrl;
  }

  // Fallback to default
  return 'https://nainee-chatbot.hf.space';
};

// Function to initialize from Docusaurus config
export const initializeBackendConfig = (customFields = {}) => {
  // Set on window object for client-side access
  if (typeof window !== 'undefined') {
    window.__BACKEND_CONFIG__ = {
      baseUrl: customFields.backendUrl || 'https://nainee-chatbot.hf.space',
    };
  }
};