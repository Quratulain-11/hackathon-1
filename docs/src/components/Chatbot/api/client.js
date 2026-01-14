/**
 * API client for communicating with the Hugging Face Spaces backend
 */

// Create a timeout promise
const timeoutPromise = (ms) => {
  return new Promise((_, reject) => {
    setTimeout(() => reject(new Error('Request timeout after ' + ms + 'ms')), ms);
  });
};

export const sendMessage = async (query, maxRetries = 1) => {
  // Use environment variable for backend URL, fallback to config
  const backendUrl = typeof window !== 'undefined' && window.location.hostname.includes('localhost')
    ? 'https://nainee-chatbot.hf.space'  // Use direct backend for local dev
    : (process.env.NEXT_PUBLIC_BACKEND_URL || 'https://nainee-chatbot.hf.space');

  // Prepare the request body in the correct Hugging Face Gradio format
  const hfRequestBody = {
    data: [query]
  };

  // Retry mechanism for cold starts
  for (let attempt = 0; attempt <= maxRetries; attempt++) {
    try {
      // Create AbortController for timeout handling
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 60000); // 60 second timeout

      const endpointUrl = `${backendUrl}/run/predict`;

      const response = await fetch(endpointUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json',
        },
        body: JSON.stringify(hfRequestBody),
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        // Handle 429 specifically for rate limiting
        if (response.status === 429) {
          return {
            answer: "The system is temporarily busy. Please try again in a moment.",
            sources: [],
            query: query,
            confidence: 0.0,
            retrieval_time_ms: 0,
            response_time_ms: 0,
            timestamp: new Date().toISOString()
          };
        }

        // Handle 405 specifically - Common during cold starts
        if (response.status === 405) {
          console.error('405 Method Not Allowed error - backend may not be fully ready');
          if (attempt < maxRetries) {
            // Wait before retrying for cold start
            await new Promise(resolve => setTimeout(resolve, 5000));
            continue;
          }
          return {
            answer: "I'm temporarily unable to process your request. Please try again in a moment.",
            sources: [],
            query: query,
            message: "The backend service is experiencing issues. Our team has been notified."
          };
        }

        // For other error statuses, try once more before giving up
        if (attempt < maxRetries) {
          await new Promise(resolve => setTimeout(resolve, 3000));
          continue;
        }

        // If it's the last attempt, return error message
        const errorText = await response.text().catch(() => '');
        console.error(`Server error: ${response.status}`, errorText);
        return {
          answer: "I'm experiencing technical difficulties. Please try again in a moment.",
          sources: [],
          query: query,
          error: `HTTP error! status: ${response.status}`
        };
      }

      // Parse response - check if it's HTML (error page) before parsing as JSON
      const contentType = response.headers.get('content-type');
      let data;

      if (contentType && contentType.includes('application/json')) {
        data = await response.json();
      } else {
        // If not JSON, it might be an HTML error page - try to handle gracefully
        const responseText = await response.text();

        // Check if response looks like JSON despite content-type
        if (responseText.trim().startsWith('{') || responseText.trim().startsWith('[')) {
          try {
            data = JSON.parse(responseText);
          } catch {
            // If it's not valid JSON, return a generic error
            if (attempt < maxRetries) {
              await new Promise(resolve => setTimeout(resolve, 3000));
              continue;
            }
            return {
              answer: "I'm experiencing technical difficulties. Please try again in a moment.",
              sources: [],
              query: query,
              error: 'Invalid response from server'
            };
          }
        } else {
          // It's HTML or plain text error page
          if (attempt < maxRetries) {
            await new Promise(resolve => setTimeout(resolve, 3000));
            continue;
          }
          return {
            answer: "I'm experiencing technical difficulties. Please try again in a moment.",
            sources: [],
            query: query,
            error: 'Received unexpected response from server'
          };
        }
      }

      // Extract the result from the response (Hugging Face format)
      let answer = '';
      if (data && Array.isArray(data.data)) {
        answer = data.data[0] || JSON.stringify(data);
      } else if (data && typeof data === 'object') {
        // Try to find the answer in different possible fields
        answer = data.answer || data.response || data.result || data.generated_text || JSON.stringify(data);
      } else {
        answer = String(data);
      }

      // Return response in the expected format
      return {
        answer: answer,
        sources: [],
        query: query,
        confidence: 0.7, // Default confidence
        retrieval_time_ms: 0,
        response_time_ms: 0,
        timestamp: new Date().toISOString()
      };
    } catch (error) {
      clearTimeout(timeoutId);

      if (error.name === 'AbortError') {
        // Timeout occurred
        if (attempt === maxRetries) {
          return {
            answer: "The request is taking too long to process. Please try again in a moment.",
            sources: [],
            query: query,
            error: 'Request timed out'
          };
        }
        // Retry on timeout
        await new Promise(resolve => setTimeout(resolve, 3000));
        continue;
      }

      console.error('Fetch error (attempt ' + attempt + '):', error.message);

      if (attempt === maxRetries) {
        return {
          answer: "I'm experiencing technical difficulties. Please try again in a moment.",
          sources: [],
          query: query,
          error: error.message
        };
      }

      // Wait before retrying
      await new Promise(resolve => setTimeout(resolve, 3000));
    }
  }

  // This line should never be reached due to the return statements above
  return {
    answer: "An unexpected error occurred. Please try again in a moment.",
    sources: [],
    query: query,
    error: 'Unexpected error in sendMessage function'
  };
};