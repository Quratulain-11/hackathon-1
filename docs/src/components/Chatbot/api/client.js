/**
 * API client for communicating with the Hugging Face Spaces backend
 */
import { BACKEND_CONFIG } from '../config';

// Create a timeout promise
const timeoutPromise = (ms) => {
  return new Promise((_, reject) => {
    setTimeout(() => reject(new Error('Request timeout after ' + ms + 'ms')), ms);
  });
};

export const sendMessage = async (query, maxRetries = 3) => {
  const request = {
    query,
    top_k: 5, // Default to 5 top results
    temperature: 0.7, // Default temperature
    max_tokens: 500, // Default max tokens
  };

  // Determine the endpoint to use based on environment
  // Use Vercel API route proxy when deployed to avoid CORS issues
  const isVercel = typeof window !== 'undefined' && window.location.hostname.includes('vercel.app');
  const isNetlify = typeof window !== 'undefined' && window.location.hostname.includes('netlify.app');
  const isProduction = typeof window !== 'undefined' && !window.location.hostname.includes('localhost');

  // Use proxy API route when deployed to Vercel or other platforms to avoid CORS
  // For Hugging Face Spaces, we need to use the proxy to avoid CORS issues
  const endpoint = (isVercel || isNetlify || (isProduction && !window.location.hostname.includes('github')))
    ? '/api/chat'  // Use Vercel API route proxy
    : `${BACKEND_CONFIG.baseUrl}/api/v1/chat`;  // Direct to Hugging Face

  // Retry mechanism for transient network errors
  for (let attempt = 0; attempt <= maxRetries; attempt++) {
    try {
      // Create the fetch promise with proper error handling
      const fetchPromise = fetch(endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // Additional headers that might be needed for Hugging Face Spaces
          'Accept': 'application/json',
        },
        body: JSON.stringify(request),
      });

      // Race the fetch promise against the timeout
      const response = await Promise.race([
        fetchPromise,
        timeoutPromise(30000) // 30 second timeout
      ]);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        const errorMessage = errorData.detail || errorData.error || `HTTP error! status: ${response.status}`;

        // Handle 429 specifically for rate limiting
        if (response.status === 429) {
          // Return a user-friendly message instead of throwing an error
          return {
            answer: "The system is temporarily busy. Please try again in a moment.",
            sources: [],
            query: request.query,
            confidence: 0.0,
            retrieval_time_ms: 0,
            response_time_ms: 0,
            timestamp: new Date().toISOString()
          };
        }

        // Handle 405 specifically - this often means the backend is not ready yet
        if (response.status === 405) {
          console.error('405 Method Not Allowed error - backend may not be fully ready:', errorMessage);
          throw new Error(`Backend service not ready: ${errorMessage}. This may be a temporary issue with the Hugging Face Space waking up. Please try again in a moment.`);
        }

        // Don't retry on 4xx client errors (except 429 and 405 which are handled above)
        if (response.status >= 400 && response.status < 500 && response.status !== 429 && response.status !== 405) {
          console.error('Client error:', errorMessage);
          throw new Error(errorMessage);
        }

        console.error('Server error:', errorMessage);
        throw new Error(errorMessage);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Fetch error (attempt ' + attempt + '):', error.message);

      // If it's a timeout error
      if (error instanceof Error && error.message.includes('timeout')) {
        if (attempt === maxRetries) {
          throw new Error('Request timed out. The backend is taking too long to respond.');
        }
        // Wait before retrying (exponential backoff)
        await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
        continue;
      }

      // Handle network errors that might be transient
      if (error instanceof TypeError && error.message.includes('fetch')) {
        if (attempt === maxRetries) {
          throw new Error('Unable to connect to the backend. Please ensure the Hugging Face Spaces backend is running.');
        }
        // Wait before retrying (exponential backoff)
        await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
        continue;
      }

      // Handle 429 (Too Many Requests) errors with retry
      if (error instanceof Error && error.message.includes('429')) {
        if (attempt === maxRetries) {
          throw error;
        }
        // Wait before retrying (exponential backoff)
        await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
        continue;
      }

      // Handle other errors
      if (error instanceof Error) {
        // If it's the last attempt, throw the error
        if (attempt === maxRetries) {
          console.error('Final error after retries:', error.message);
          throw error;
        }
        // Otherwise, wait before retrying (exponential backoff)
        await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
        continue;
      }

      // If it's the last attempt, throw the error
      if (attempt === maxRetries) {
        throw new Error('An unknown error occurred while sending the message.');
      }
      // Otherwise, wait before retrying (exponential backoff)
      await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
    }
  }

  // This line should never be reached due to the return statements above
  throw new Error('Unexpected error in sendMessage function');
};