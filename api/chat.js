// Vercel API route to proxy requests to the backend
// This helps avoid CORS issues when deployed to Vercel
export default async function handler(req, res) {
  try {
    // Determine allowed origin based on environment
    const allowedOrigin = process.env.ALLOWED_ORIGIN ||
                          process.env.NEXT_PUBLIC_ALLOWED_ORIGIN ||
                          process.env.VERCEL_URL ?
                          `https://${process.env.VERCEL_URL}` :
                          req.headers.origin || '*';

    // Enable CORS with specific origin
    res.setHeader('Access-Control-Allow-Credentials', true);
    res.setHeader('Access-Control-Allow-Origin', allowedOrigin);
    res.setHeader('Access-Control-Allow-Methods', 'GET,OPTIONS,PATCH,DELETE,POST,PUT');
    res.setHeader(
      'Access-Control-Allow-Headers',
      'X-CSRF-Token, X-Requested-With, Accept, Accept-Version, Content-Length, Content-MD5, Content-Type, Date, X-Api-Version'
    );

    // Handle preflight requests
    if (req.method === 'OPTIONS') {
      res.status(200).end();
      return;
    }

    // Only allow POST requests (for actual chat requests)
    if (req.method !== 'POST') {
      console.log('Method not allowed:', req.method);
      return res.status(200).json({
        answer: "I'm temporarily unable to process your request. Please try again in a moment.",
        sources: [],
        query: req.body?.query || req.body?.message || '',
        message: "Invalid request method. Only POST requests are allowed."
      });
    }

    // Validate request body
    if (!req.body || Object.keys(req.body).length === 0) {
      console.log('Empty request body received');
      return res.status(200).json({
        answer: "I'm temporarily unable to process your request. Please try again in a moment.",
        sources: [],
        query: '',
        message: "Request body is required"
      });
    }

    // Get the backend URL from environment variables
    const backendUrl = process.env.BACKEND_URL || process.env.NEXT_PUBLIC_BACKEND_URL || 'https://nainee-chatbot.hf.space';

    console.log('Backend URL:', backendUrl);
    console.log('Request body:', req.body);

    // Function to warm up the Hugging Face Space with better error handling
    const warmUpSpace = async (maxRetries = 3) => {
      for (let attempt = 0; attempt < maxRetries; attempt++) {
        try {
          console.log(`Warming up Hugging Face Space (attempt ${attempt + 1}/${maxRetries})...`);

          // Try the health endpoint first
          const healthUrl = `${backendUrl}/health`;
          let healthResponse;
          try {
            healthResponse = await fetch(healthUrl, {
              method: 'GET',
              headers: {
                'User-Agent': 'Vercel-Proxy/1.0',
                'Accept': 'application/json'
              }
            });
          } catch (networkError) {
            console.log(`Health check network error (attempt ${attempt + 1}):`, networkError.message);
            if (attempt === maxRetries - 1) {
              console.log('All health check attempts failed, continuing anyway...');
              return; // Continue even if health check fails
            }
            await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000)); // Exponential backoff
            continue;
          }

          console.log(`Health check status: ${healthResponse.status}`);

          if (healthResponse.status >= 200 && healthResponse.status < 300) {
            // Health check successful, space is ready
            return;
          } else if (healthResponse.status === 404 || healthResponse.status === 405) {
            // If health endpoint doesn't exist, try the root
            const rootResponse = await fetch(backendUrl, {
              method: 'GET',
              headers: {
                'User-Agent': 'Vercel-Proxy/1.0',
                'Accept': 'application/json'
              }
            });
            console.log(`Root check status: ${rootResponse.status}`);

            if (rootResponse.status >= 200 && rootResponse.status < 300) {
              return; // Root is accessible, space is likely ready
            }
          }

          // Wait for space to wake up (with exponential backoff)
          await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
        } catch (warmUpError) {
          console.log(`Warm-up attempt ${attempt + 1} failed:`, warmUpError.message);
          if (attempt === maxRetries - 1) {
            console.log('All warm-up attempts failed, continuing anyway...');
          } else {
            // Wait before next attempt
            await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
          }
        }
      }
    };

    // Warm up the space before making the main request
    await warmUpSpace();

    // Prepare the request body in the correct Hugging Face format
    const userMessage = req.body.query || req.body.message || req.body.input || JSON.stringify(req.body);

    // Validate user message
    if (!userMessage || typeof userMessage !== 'string' || userMessage.trim() === '') {
      return res.status(200).json({
        answer: "I'm temporarily unable to process your request. Please try again in a moment.",
        sources: [],
        query: '',
        message: "Valid query is required"
      });
    }

    const hfRequestBody = {
      data: [userMessage.trim()]
    };

    // Validate payload
    if (!hfRequestBody.data || !Array.isArray(hfRequestBody.data) || hfRequestBody.data.length === 0) {
      return res.status(200).json({
        answer: "I'm temporarily unable to process your request. Please try again in a moment.",
        sources: [],
        query: userMessage,
        message: "Invalid request payload format"
      });
    }

    // Use the correct API endpoint for predictions
    const endpointUrl = `${backendUrl}/api/v1/chat`;
    console.log(`Attempting to call: ${endpointUrl}`);
    console.log(`Sending request body:`, hfRequestBody);

    // Prepare the correct API request body format
    const apiRequestBody = {
      query: userMessage.trim(),
      top_k: 5,
      temperature: 0.7,
      max_tokens: 500
    };

    // Exponential backoff retry mechanism for cold starts
    const maxRetries = 2;
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
              'User-Agent': 'Vercel-Proxy/1.0',
              'Connection': 'keep-alive',
              'Accept-Encoding': 'gzip, deflate, br'
            },
            body: JSON.stringify(apiRequestBody),
            signal: controller.signal
          });
        } catch (networkError) {
          clearTimeout(timeoutId);

          if (attempt < maxRetries) {
            const delay = Math.pow(2, attempt) * 1000; // Exponential backoff
            await new Promise(resolve => setTimeout(resolve, delay));
            continue;
          }

          console.error('Network error when calling backend:', networkError.message);
          return res.status(200).json({
            answer: "I'm temporarily unable to process your request. Please try again in a moment.",
            sources: [],
            query: userMessage,
            message: "Network error connecting to backend service."
          });
        }

        clearTimeout(timeoutId);

        // Handle 429 specifically for rate limiting
        if (response.status === 429) {
          return res.status(200).json({
            answer: "The system is temporarily busy. Please try again in a moment.",
            sources: [],
            query: userMessage,
            confidence: 0.0,
            retrieval_time_ms: 0,
            response_time_ms: 0,
            timestamp: new Date().toISOString()
          });
        }

        // Handle 405 and 502 as cold-start signals
        if (response.status === 405 || response.status === 502 || response.status === 503) {
          console.warn(`Cold start detected (status: ${response.status}), retrying... (attempt ${attempt + 1}/${maxRetries + 1})`);
          if (attempt < maxRetries) {
            const delay = Math.pow(2, attempt) * 2000; // Exponential backoff for cold starts
            await new Promise(resolve => setTimeout(resolve, delay));
            continue;
          }

          return res.status(200).json({
            answer: "I'm temporarily unable to process your request. Please try again in a moment.",
            sources: [],
            query: userMessage,
            message: "The backend service is experiencing issues. Our team has been notified."
          });
        }

        // For other error statuses, try again before giving up
        if (!response.ok) {
          console.log(`Received error status ${response.status}, attempting retry... (attempt ${attempt + 1}/${maxRetries + 1})`);
          if (attempt < maxRetries) {
            const delay = Math.pow(2, attempt) * 1000;
            await new Promise(resolve => setTimeout(resolve, delay));
            continue;
          }

          // If it's the last attempt, return error message
          const errorText = await response.text().catch(() => '');
          console.error(`Server error: ${response.status}`, errorText);
          return res.status(200).json({
            answer: "I'm experiencing technical difficulties. Please try again in a moment.",
            sources: [],
            query: userMessage,
            error: `HTTP error! status: ${response.status}`
          });
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

          return res.status(200).json({
            answer: "I'm experiencing technical difficulties. Please try again in a moment.",
            sources: [],
            query: userMessage,
            error: `Failed to read response: ${textError.message}`
          });
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

          return res.status(200).json({
            answer: "I'm experiencing technical difficulties. Please try again in a moment.",
            sources: [],
            query: userMessage,
            error: 'Received HTML response instead of JSON'
          });
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

              return res.status(200).json({
                answer: "I'm experiencing technical difficulties. Please try again in a moment.",
                sources: [],
                query: userMessage,
                error: 'Invalid JSON response from server'
              });
            }
          } else {
            // It's an HTML or plain text error page
            if (attempt < maxRetries) {
              const delay = Math.pow(2, attempt) * 2000; // Exponential backoff for cold starts
              await new Promise(resolve => setTimeout(resolve, delay));
              continue;
            }

            return res.status(200).json({
              answer: "I'm experiencing technical difficulties. Please try again in a moment.",
              sources: [],
              query: userMessage,
              error: 'Received unexpected response format from server'
            });
          }
        }

        // Validate response shape before accessing data
        if (!data) {
          if (attempt < maxRetries) {
            const delay = Math.pow(2, attempt) * 1000;
            await new Promise(resolve => setTimeout(resolve, delay));
            continue;
          }

          return res.status(200).json({
            answer: "I'm experiencing technical difficulties. Please try again in a moment.",
            sources: [],
            query: userMessage,
            error: 'Empty response from server'
          });
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

        // Return the response from the backend
        // Use Math.min to ensure we don't return an invalid status code
        const statusCode = Math.min(Math.max(response.status, 100), 599);
        console.log(`Returning successful response with status: ${statusCode}`);
        return res.status(200).json({  // Changed to always return 200 to avoid client-side errors
          answer: answer || "I'm unable to process your request at the moment. Please try again later.",
          sources: Array.isArray(data.sources) ? data.sources : [],
          query: userMessage,
          confidence: typeof data.confidence === 'number' ? data.confidence : 0.7,
          retrieval_time_ms: typeof data.retrieval_time_ms === 'number' ? data.retrieval_time_ms : 0,
          response_time_ms: typeof data.response_time_ms === 'number' ? data.response_time_ms : 0,
          timestamp: data.timestamp || new Date().toISOString()
        });
      } catch (innerError) {
        console.error('Inner error in API handler (attempt ' + attempt + '):', innerError);

        if (attempt < maxRetries) {
          const delay = Math.pow(2, attempt) * 1000;
          await new Promise(resolve => setTimeout(resolve, delay));
          continue;
        }

        return res.status(200).json({
          answer: "I'm experiencing technical difficulties. Please try again in a moment.",
          sources: [],
          query: userMessage,
          error: innerError.message
        });
      }
    }

    // This should not be reached due to the return statements above, but as a safeguard:
    return res.status(200).json({
      answer: "An unexpected error occurred. Please try again in a moment.",
      sources: [],
      query: userMessage,
      error: 'Unexpected error in API handler'
    });
  } catch (outerError) {
    console.error('Outer error in API handler:', outerError);

    // Return a friendly error message instead of propagating the error
    return res.status(200).json({
      error: 'Backend service temporarily unavailable',
      details: 'Please try again in a moment',
      answer: "I'm currently unable to process your request. Please try again in a moment.",
      sources: [],
      query: '',
      message: 'The backend service is experiencing issues. Our team has been notified.'
    });
  }
}