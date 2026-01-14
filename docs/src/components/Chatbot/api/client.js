/**
 * API client for communicating with the Hugging Face Spaces backend
 */

// Access environment variable at module level to avoid runtime process checks
const ENV_BACKEND_URL = typeof window !== 'undefined' && window.ENV && window.ENV.NEXT_PUBLIC_BACKEND_URL
  ? window.ENV.NEXT_PUBLIC_BACKEND_URL
  : (typeof window !== 'undefined' && window.env && window.env.NEXT_PUBLIC_BACKEND_URL
    ? window.env.NEXT_PUBLIC_BACKEND_URL
    : undefined);

// Centralized backend URL resolution
const getBackendUrl = () => {
  // Validate and return backend URL
  const defaultUrl = 'https://nainee-chatbot.hf.space';
  const envUrl = ENV_BACKEND_URL;

  // If we have an environment variable and it's a valid URL, use it
  if (envUrl && typeof envUrl === 'string' && envUrl.trim() !== '') {
    try {
      // Ensure the URL is properly formatted
      const urlObj = new URL(envUrl);
      return urlObj.href.replace(/\/$/, ''); // Remove trailing slash if present
    } catch (e) {
      console.warn('Invalid BACKEND_URL from environment, using default:', envUrl);
      return defaultUrl;
    }
  }

  return defaultUrl;
};

export const sendMessage = async (query, maxRetries = 2) => {
  try {
    // Validate input
    if (!query || typeof query !== 'string' || query.trim() === '') {
      return {
        answer: "I'm unable to process your request at the moment. Please try again later.",
        sources: [],
        query: query || '',
        error: 'Invalid or empty query provided'
      };
    }

    // Get validated backend URL
    const backendUrl = getBackendUrl();

    // Validate endpoint format
    const endpointUrl = `${backendUrl}/run/predict`;

    // Prepare the request body in the correct Hugging Face Gradio format
    const hfRequestBody = {
      data: [query.trim()]
    };

    // Validate payload before sending
    if (!hfRequestBody.data || !Array.isArray(hfRequestBody.data) || hfRequestBody.data.length === 0) {
      return {
        answer: "I'm unable to process your request at the moment. Please try again later.",
        sources: [],
        query: query,
        error: 'Invalid request payload format'
      };
    }

    // Determine if we should use the Vercel proxy or direct connection
    // Use direct connection to backend to avoid potential proxy issues
    const isLocalhost = typeof window !== 'undefined' && window.location.hostname.includes('localhost');

    // For production, use direct connection to avoid proxy complications
    if (!isLocalhost) {
      // Exponential backoff retry mechanism for cold starts
      for (let attempt = 0; attempt <= maxRetries; attempt++) {
        try {
          // Create AbortController for timeout handling
          const controller = new AbortController();
          const timeoutId = setTimeout(() => controller.abort(), 60000); // 60 second timeout

          let response;
          try {
            response = await fetch(endpointUrl, {
              method: 'POST',
              headers: {
                'Content-Type': 'application/json',
                'Accept': 'application/json',
              },
              body: JSON.stringify(hfRequestBody),
              signal: controller.signal
            });
          } catch (fetchError) {
            clearTimeout(timeoutId);

            // Handle network errors
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 1000; // Exponential backoff
              await new Promise(resolve => setTimeout(resolve, delay));
              continue;
            }

            return {
              answer: "I'm experiencing technical difficulties. Please try again in a moment.",
              sources: [],
              query: query,
              error: `Network error: ${fetchError.message}`
            };
          }

          clearTimeout(timeoutId);

          // Check if response is valid
          if (!response) {
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 1000;
              await new Promise(resolve => setTimeout(resolve, delay));
              continue;
            }

            return {
              answer: "I'm experiencing technical difficulties. Please try again in a moment.",
              sources: [],
              query: query,
              error: 'Invalid response from server'
            };
          }

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

          // Handle 405 and 502 as cold-start signals
          if (response.status === 405 || response.status === 502 || response.status === 503) {
            console.warn(`Cold start detected (status: ${response.status}), retrying...`);
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 2000; // Exponential backoff
              await new Promise(resolve => setTimeout(resolve, delay));
              continue;
            }

            return {
              answer: "I'm temporarily unable to process your request. Please try again in a moment.",
              sources: [],
              query: query,
              message: "The backend service is experiencing issues. Our team has been notified."
            };
          }

          // For other error statuses, try again before giving up
          if (!response.ok) {
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 1000;
              await new Promise(resolve => setTimeout(resolve, delay));
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

          // Safely parse response - check if it's HTML (error page) before parsing as JSON
          let responseText;
          try {
            responseText = await response.text();
          } catch (textError) {
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 1000;
              await new Promise(resolve => setTimeout(resolve, delay));
              continue;
            }

            return {
              answer: "I'm experiencing technical difficulties. Please try again in a moment.",
              sources: [],
              query: query,
              error: `Failed to read response: ${textError.message}`
            };
          }

          // Block HTML responses before JSON parsing to prevent "Unexpected token <" errors
          if (responseText.trim().startsWith('<!DOCTYPE') ||
              responseText.trim().startsWith('<html') ||
              responseText.trim().startsWith('<')) {
            console.warn('HTML response detected, treating as cold start error');
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 2000; // Exponential backoff for cold starts
              await new Promise(resolve => setTimeout(resolve, delay));
              continue;
            }

            return {
              answer: "I'm experiencing technical difficulties. Please try again in a moment.",
              sources: [],
              query: query,
              error: 'Received HTML response instead of JSON'
            };
          }

          let data;
          try {
            data = JSON.parse(responseText);
          } catch (parseError) {
            // Check if response looks like JSON despite content-type
            if (responseText.trim().startsWith('{') || responseText.trim().startsWith('[')) {
              try {
                data = JSON.parse(responseText);
              } catch {
                if (attempt < maxRetries) {
                  const delay = Math.pow(2, attempt) * 1000;
                  await new Promise(resolve => setTimeout(resolve, delay));
                  continue;
                }

                return {
                  answer: "I'm experiencing technical difficulties. Please try again in a moment.",
                  sources: [],
                  query: query,
                  error: 'Invalid JSON response from server'
                };
              }
            } else {
              // It's an HTML or plain text error page
              if (attempt < maxRetries) {
                const delay = Math.pow(2, attempt) * 2000; // Exponential backoff for cold starts
                await new Promise(resolve => setTimeout(resolve, delay));
                continue;
              }

              return {
                answer: "I'm experiencing technical difficulties. Please try again in a moment.",
                sources: [],
                query: query,
                error: 'Received unexpected response format from server'
              };
            }
          }

          // Validate response shape before accessing data
          if (!data) {
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 1000;
              await new Promise(resolve => setTimeout(resolve, delay));
              continue;
            }

            return {
              answer: "I'm experiencing technical difficulties. Please try again in a moment.",
              sources: [],
              query: query,
              error: 'Empty response from server'
            };
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
            answer: answer || "I'm unable to process your request at the moment. Please try again later.",
            sources: Array.isArray(data.sources) ? data.sources : [],
            query: query,
            confidence: typeof data.confidence === 'number' ? data.confidence : 0.7,
            retrieval_time_ms: typeof data.retrieval_time_ms === 'number' ? data.retrieval_time_ms : 0,
            response_time_ms: typeof data.response_time_ms === 'number' ? data.response_time_ms : 0,
            timestamp: data.timestamp || new Date().toISOString()
          };
        } catch (innerError) {
          // Catch any errors in the try block and handle appropriately
          if (attempt < maxRetries) {
            const delay = Math.pow(2, attempt) * 1000;
            await new Promise(resolve => setTimeout(resolve, delay));
            continue;
          }

          console.error('Inner error in sendMessage (attempt ' + attempt + '):', innerError);
          return {
            answer: "I'm experiencing technical difficulties. Please try again in a moment.",
            sources: [],
            query: query,
            error: innerError.message
          };
        }
      }
    } else {
      // For localhost development, use the Vercel proxy to avoid CORS issues
      const proxyEndpoint = '/api/chat';

      for (let attempt = 0; attempt <= maxRetries; attempt++) {
        try {
          const controller = new AbortController();
          const timeoutId = setTimeout(() => controller.abort(), 60000);

          const response = await fetch(proxyEndpoint, {
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
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 1000;
              await new Promise(resolve => setTimeout(resolve, delay));
              continue;
            }

            const errorData = await response.json().catch(() => ({}));
            const errorMessage = errorData.error || errorData.message || `HTTP error! status: ${response.status}`;

            return {
              answer: "I'm experiencing technical difficulties. Please try again in a moment.",
              sources: [],
              query: query,
              error: errorMessage
            };
          }

          const result = await response.json();

          // Validate the response structure
          if (!result || typeof result !== 'object') {
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 1000;
              await new Promise(resolve => setTimeout(resolve, delay));
              continue;
            }

            return {
              answer: "I'm experiencing technical difficulties. Please try again in a moment.",
              sources: [],
              query: query,
              error: 'Invalid response structure from server'
            };
          }

          return {
            answer: result.answer || result.response || "I'm unable to process your request at the moment. Please try again later.",
            sources: Array.isArray(result.sources) ? result.sources : [],
            query: query,
            confidence: typeof result.confidence === 'number' ? result.confidence : 0.7,
            retrieval_time_ms: typeof result.retrieval_time_ms === 'number' ? result.retrieval_time_ms : 0,
            response_time_ms: typeof result.response_time_ms === 'number' ? result.response_time_ms : 0,
            timestamp: result.timestamp || new Date().toISOString()
          };
        } catch (fetchError) {
          clearTimeout(timeoutId);

          if (fetchError.name === 'AbortError') {
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 1000;
              await new Promise(resolve => setTimeout(resolve, delay));
              continue;
            }

            return {
              answer: "I'm experiencing technical difficulties. Please try again in a moment.",
              sources: [],
              query: query,
              error: 'Request timeout'
            };
          }

          if (attempt < maxRetries) {
            const delay = Math.pow(2, attempt) * 1000;
            await new Promise(resolve => setTimeout(resolve, delay));
            continue;
          }

          return {
            answer: "I'm experiencing technical difficulties. Please try again in a moment.",
            sources: [],
            query: query,
            error: fetchError.message
          };
        }
      }
    }

    // This should not be reached due to the return statements above, but as a safeguard:
    return {
      answer: "An unexpected error occurred. Please try again in a moment.",
      sources: [],
      query: query,
      error: 'Unexpected error in sendMessage function'
    };
  } catch (outerError) {
    // Final catch-all to ensure no uncaught exceptions reach the UI
    console.error('Outer error in sendMessage:', outerError);
    return {
      answer: "I'm experiencing technical difficulties. Please try again in a moment.",
      sources: [],
      query: query,
      error: outerError.message
    };
  }
};