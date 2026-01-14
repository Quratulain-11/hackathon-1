/**
 * Configuration for the Chatbot component
 */
// Access environment variable at module level to avoid runtime process checks
const ENV_BASE_URL = typeof window !== 'undefined' && window.ENV && window.ENV.NEXT_PUBLIC_BACKEND_URL
  ? window.ENV.NEXT_PUBLIC_BACKEND_URL
  : (typeof window !== 'undefined' && window.env && window.env.NEXT_PUBLIC_BACKEND_URL
    ? window.env.NEXT_PUBLIC_BACKEND_URL
    : undefined);

export const getBackendConfig = () => {
  const defaultBaseUrl = 'https://nainee-chatbot.hf.space';

  // Check if we have a configured URL in window (set by Docusaurus)
  if (typeof window !== 'undefined' && window.__BACKEND_CONFIG__) {
    return {
      baseUrl: ENV_BASE_URL || window.__BACKEND_CONFIG__.baseUrl || defaultBaseUrl
    };
  }

  // For SSR or when window is not available, use environment variable or default
  return {
    baseUrl: ENV_BASE_URL || defaultBaseUrl
  };
};

// Maintain the original export for compatibility
export const BACKEND_CONFIG = getBackendConfig();