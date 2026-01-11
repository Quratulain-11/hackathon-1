/**
 * Configuration for the Chatbot component
 */
// Dynamic configuration that avoids process.env usage
export const getBackendConfig = () => {
  // Use a default URL that can be overridden via window object
  const defaultBaseUrl = 'https://nainee-chatbot.hf.space';

  // Check if we have a configured URL in window (set by Docusaurus)
  if (typeof window !== 'undefined' && window.__BACKEND_CONFIG__) {
    return {
      baseUrl: window.__BACKEND_CONFIG__.baseUrl || defaultBaseUrl
    };
  }

  // For SSR or when window is not available, use default
  return {
    baseUrl: defaultBaseUrl
  };
};

// Maintain the original export for compatibility
export const BACKEND_CONFIG = getBackendConfig();