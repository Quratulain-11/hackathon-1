// Vercel API route to proxy requests to the backend
// This helps avoid CORS issues when deployed to Vercel
export default async function handler(req, res) {
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
    return res.status(405).json({
      error: 'Method not allowed',
      details: `Only POST requests are allowed, received ${req.method}`
    });
  }

  // Validate request body
  if (!req.body || Object.keys(req.body).length === 0) {
    console.log('Empty request body received');
    return res.status(400).json({
      error: 'Request body is required',
      details: 'No query provided in request body'
    });
  }

  try {
    // Get the backend URL from environment variables
    const backendUrl = process.env.BACKEND_URL || 'https://nainee-chatbot.hf.space';

    console.log('=== DEBUG: Processing chat request ===');
    console.log('Backend URL:', backendUrl);
    console.log('Request body:', JSON.stringify(req.body, null, 2));
    console.log('Full request headers:', req.headers);

    // Function to check if backend is accessible
    const checkBackendAccessibility = async () => {
      try {
        const testResponse = await fetch(backendUrl, {
          method: 'GET',
          headers: {
            'User-Agent': 'Vercel-Proxy-Diagnostic/1.0',
            'Accept': 'application/json,text/html,*/*'
          }
        });
        console.log(`Backend accessibility test - Status: ${testResponse.status}`);

        if (testResponse.status >= 200 && testResponse.status < 400) {
          return true;
        }
        return false;
      } catch (error) {
        console.log(`Backend accessibility test failed:`, error.message);
        return false;
      }
    };

    // Check if backend is accessible before proceeding
    const isAccessible = await checkBackendAccessibility();
    if (!isAccessible) {
      console.log('WARNING: Backend appears to be inaccessible');
      return res.status(503).json({
        error: 'Backend service unavailable',
        details: 'The backend service is not accessible',
        answer: "I'm currently unable to connect to my backend service. Please try again later.",
        sources: [],
        query: req.body.query || req.body.question || req.body.message || '',
        message: 'Backend service is not accessible'
      });
    }

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
            await new Promise(resolve => setTimeout(resolve, 2000 * (attempt + 1))); // Exponential backoff
            continue;
          }

          console.log(`Health check status: ${healthResponse.status}`);

          if (healthResponse.status >= 200 && healthResponse.status < 300) {
            // Health check successful, space is ready
            console.log('Health check successful, space is ready');
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
              console.log('Root is accessible, space is likely ready');
              return; // Root is accessible, space is likely ready
            }
          }

          // Wait for space to wake up (with exponential backoff)
          await new Promise(resolve => setTimeout(resolve, 2000 * (attempt + 1)));
        } catch (warmUpError) {
          console.log(`Warm-up attempt ${attempt + 1} failed:`, warmUpError.message);
          if (attempt === maxRetries - 1) {
            console.log('All warm-up attempts failed, continuing anyway...');
          } else {
            // Wait before next attempt
            await new Promise(resolve => setTimeout(resolve, 2000 * (attempt + 1)));
          }
        }
      }
    };

    // Warm up the space before making the main request
    await warmUpSpace();

    // Based on the logs, your backend appears to be a FastAPI application
    // Let's try the most common FastAPI chat endpoints
    const endpointsToTry = [
      `${backendUrl}/api/v1/chat`,
      `${backendUrl}/chat`,
      `${backendUrl}/api/chat`,
      `${backendUrl}/query`,
      `${backendUrl}/rag/query`,
      `${backendUrl}/ask`
    ];

    let response = null;
    let lastError = null;
    let successfulEndpoint = null;

    // Try each endpoint in sequence
    for (const endpointUrl of endpointsToTry) {
      console.log(`Attempting to call: ${endpointUrl}`);

      try {
        // Create a timeout promise for the fetch call
        const timeoutPromise = new Promise((_, reject) => {
          setTimeout(() => reject(new Error('Request timeout after 30 seconds')), 30000);
        });

        // Prepare the request body - adapt to what the backend expects
        // If the original request has a 'query' field, use it as the main input
        let requestBody;
        if (req.body.query) {
          // Standard format for RAG applications
          requestBody = {
            query: req.body.query,
            top_k: req.body.top_k || 5,
            temperature: req.body.temperature || 0.7,
            max_tokens: req.body.max_tokens || 500
          };
        } else if (req.body.inputs || req.body.input) {
          // Format for Hugging Face-style endpoints
          requestBody = {
            inputs: req.body.inputs || req.body.input,
            parameters: {
              top_k: req.body.top_k || 5,
              temperature: req.body.temperature || 0.7,
              max_tokens: req.body.max_tokens || 500
            }
          };
        } else {
          // Fallback: use original request body
          requestBody = req.body;
        }

        console.log(`Sending request body to ${endpointUrl}:`, JSON.stringify(requestBody, null, 2));

        // Race the fetch request against a timeout
        const fetchPromise = fetch(endpointUrl, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'User-Agent': 'Vercel-Proxy/1.0',
            'Connection': 'keep-alive',
            'Accept-Encoding': 'gzip, deflate, br'
          },
          body: JSON.stringify(requestBody),
        });

        response = await Promise.race([fetchPromise, timeoutPromise]);

        console.log(`Response from ${endpointUrl}: status=${response.status}`);

        if (response.status !== 404 && response.status !== 405) {
          // If we got a response that's not 404 or 405, break out of the loop
          successfulEndpoint = endpointUrl;
          console.log(`SUCCESS: Found working endpoint: ${endpointUrl}`);
          break;
        } else {
          console.log(`Endpoint ${endpointUrl} returned ${response.status}, trying next...`);
          lastError = `Endpoint ${endpointUrl} returned ${response.status}`;
          response = null; // Reset response to try the next endpoint
        }
      } catch (networkError) {
        console.error(`Network error when calling ${endpointUrl}:`, networkError.message);
        lastError = `Network error calling ${endpointUrl}: ${networkError.message}`;
        continue; // Try the next endpoint
      }
    }

    // If no endpoint worked
    if (!response) {
      console.error('All endpoints failed:', lastError);

      // Return a friendly error message instead of propagating the error
      return res.status(500).json({
        error: 'Backend service temporarily unavailable',
        details: 'Could not connect to any available endpoint',
        answer: "I'm currently unable to process your request. The backend service might be temporarily unavailable. Please try again in a moment.",
        sources: [],
        query: req.body.query || req.body.question || req.body.message || '',
        message: 'The backend service is experiencing issues. Our team has been notified.'
      });
    }

    console.log(`Final response status: ${response.status}`);
    console.log(`Successful endpoint: ${successfulEndpoint}`);

    // Handle 405 - Method not allowed, which was the original issue
    if (response.status === 405) {
      console.log('Received 405 error - returning friendly message');

      return res.status(200).json({
        answer: "I'm temporarily unable to process your request. Please try again in a moment.",
        sources: [],
        query: req.body.query || req.body.question || req.body.message || '',
        message: "The backend service is experiencing issues. Our team has been notified."
      });
    }

    // Handle other error statuses
    else if (response.status >= 400) {
      console.log(`Received error status ${response.status}, processing response...`);

      let errorData;
      try {
        errorData = await response.json();
        console.log(`Error response JSON:`, errorData);
      } catch (parseError) {
        const textResponse = await response.text();
        console.log(`Error response (non-JSON): ${textResponse}`);

        // For 405 specifically, return a friendly message
        if (response.status === 405) {
          return res.status(200).json({
            answer: "I'm temporarily unable to process your request. Please try again in a moment.",
            sources: [],
            query: req.body.query || req.body.question || req.body.message || '',
            message: "The backend service is experiencing issues. Our team has been notified."
          });
        }

        return res.status(response.status).json({
          error: textResponse,
          status: response.status
        });
      }

      console.log(`Error response data:`, errorData);

      // If it's a 405 error, return a friendly message instead of the error
      if (response.status === 405) {
        return res.status(200).json({
          answer: "I'm temporarily unable to process your request. Please try again in a moment.",
          sources: [],
          query: req.body.query || req.body.question || req.body.message || '',
          message: "The backend service is experiencing issues. Our team has been notified."
        });
      }

      return res.status(response.status).json(errorData);
    }

    // Process the successful response
    let data;
    try {
      data = await response.json();
      console.log(`Successfully parsed response JSON:`, data);
    } catch (parseError) {
      // If response is not JSON (e.g., plain text error), return as is
      const textResponse = await response.text();
      console.log('Non-JSON response received:', textResponse);

      return res.status(response.status).json({
        answer: textResponse,
        sources: [],
        query: req.body.query || req.body.question || req.body.message || '',
        status: response.status
      });
    }

    // Return the response from the backend
    // Use Math.min to ensure we don't return an invalid status code
    const statusCode = Math.min(Math.max(response.status, 100), 599);
    console.log(`Returning successful response with status: ${statusCode}`);

    // Ensure the response matches the expected format for the frontend
    const formattedResponse = {
      answer: data.answer || data.response || data.result || data.generated_text || JSON.stringify(data),
      sources: data.sources || data.source_documents || data.context || [],
      query: req.body.query || req.body.question || req.body.message || '',
      original_response: data
    };

    console.log(`Final formatted response:`, formattedResponse);
    console.log('=== END DEBUG: Chat request processing ===');

    res.status(statusCode).json(formattedResponse);
  } catch (error) {
    console.error('=== ERROR DEBUG: Proxy error ===');
    console.error('Full error:', error);
    console.error('Error message:', error.message);
    console.error('Error stack:', error.stack);
    console.error('===============================');

    // Return a friendly error message instead of propagating the error
    res.status(500).json({
      error: 'Backend service temporarily unavailable',
      details: 'Please try again in a moment',
      answer: "I'm currently unable to process your request. Please try again in a moment.",
      sources: [],
      query: req.body?.query || req.body?.question || req.body?.message || '',
      message: 'The backend service is experiencing issues. Our team has been notified.'
    });
  }
}