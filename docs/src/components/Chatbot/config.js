/**
 * Configuration for the Chatbot component
 */
// Dynamic configuration that uses environment variables
export const getBackendConfig = () => {
  // Use environment variable for backend URL, fallback to default
  const envBaseUrl = typeof process !== 'undefined' ? process.env.NEXT_PUBLIC_BACKEND_URL : undefined;
  const defaultBaseUrl = 'https://nainee-chatbot.hf.space';

  // Check if we have a configured URL in window (set by Docusaurus)
  if (typeof window !== 'undefined' && window.__BACKEND_CONFIG__) {
    return {
      baseUrl: envBaseUrl || window.__BACKEND_CONFIG__.baseUrl || defaultBaseUrl
    };
  }

  // For SSR or when window is not available, use environment variable or default
  return {
    baseUrl: envBaseUrl || defaultBaseUrl
  };
};

// Maintain the original export for compatibility
export const BACKEND_CONFIG = getBackendConfig();