/**
 * Utility functions for the Chatbot component
 */

// Debounce function to limit API calls
export const debounce = (func, wait) => {
  let timeout;
  return function executedFunction(...args) {
    const later = () => {
      clearTimeout(timeout);
      func(...args);
    };
    clearTimeout(timeout);
    timeout = setTimeout(later, wait);
  };
};

// Format error messages for better user experience
export const formatErrorMessage = (error) => {
  if (error?.message?.includes('fetch')) {
    return 'Unable to connect to the backend service. The chatbot may be temporarily unavailable.';
  }
  if (error?.message?.includes('401') || error?.message?.includes('API')) {
    return 'Authentication error. Please try again later.';
  }
  if (error?.message?.includes('429')) {
    return 'The system is temporarily busy. Please try again in a moment.';
  }
  if (error?.message?.includes('503')) {
    return 'Backend service is temporarily unavailable. Please try again later.';
  }
  if (error?.message?.includes('connect to the backend')) {
    return 'The chatbot is temporarily unavailable. Please try again in a moment.';
  }
  return error?.message || 'An error occurred while processing your request.';
};