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
    const backendUrl = process.env.BACKEND_URL || process.env.REACT_APP_BACKEND_URL || process.env.NEXT_PUBLIC_BACKEND_URL || 'https://nainee-chatbot.hf.space';

    console.log('Backend URL:', backendUrl);
    console.log('Request body:', req.body);
    console.log('Request headers:', {
      'content-type': req.headers['content-type'],
      'user-agent': req.headers['user-agent'],
      'accept': req.headers['accept']
    });

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

    // Try the documented endpoint: /api/v1/chat
    const endpointUrl = `${backendUrl}/api/v1/chat`;
    console.log(`Attempting to call: ${endpointUrl}`);

    // Try with additional headers that might be required by Hugging Face
    // Create a timeout promise for the fetch call
    const timeoutPromise = new Promise((_, reject) => {
      setTimeout(() => reject(new Error('Request timeout after 30 seconds')), 30000);
    });

    let response;
    try {
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
        body: JSON.stringify(req.body),
      });

      response = await Promise.race([fetchPromise, timeoutPromise]);
    } catch (networkError) {
      console.error('Network error when calling backend:', networkError.message);
      if (networkError.message.includes('timeout')) {
        return res.status(408).json({
          error: 'Request timeout',
          details: 'Backend service took too long to respond'
        });
      }
      return res.status(502).json({
        error: 'Network error connecting to backend service',
        details: networkError.message
      });
    }

    console.log(`Response status: ${response.status}`);

    // Handle different response statuses appropriately
    if (response.status === 404) {
      console.log('Received 404 error, trying alternative endpoints...');

      // Try the /chat endpoint (might be served without /api/v1 prefix in some deployments)
      const altEndpointUrl = `${backendUrl}/chat`;
      console.log(`Trying alternative endpoint: ${altEndpointUrl}`);

      const altTimeoutPromise = new Promise((_, reject) => {
        setTimeout(() => reject(new Error('Alternative request timeout after 30 seconds')), 30000);
      });

      let altResponse;
      try {
        // Race the fetch request against a timeout
        const altFetchPromise = fetch(altEndpointUrl, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'User-Agent': 'Vercel-Proxy/1.0'
          },
          body: JSON.stringify(req.body),
        });

        altResponse = await Promise.race([altFetchPromise, altTimeoutPromise]);
      } catch (networkError) {
        console.error('Network error when calling alternative endpoint:', networkError.message);
        if (networkError.message.includes('timeout')) {
          return res.status(408).json({
            error: 'Request timeout',
            details: 'Alternative backend service took too long to respond'
          });
        }
        return res.status(502).json({
          error: 'Network error connecting to alternative endpoint',
          details: networkError.message
        });
      }

      console.log(`Alternative endpoint response status: ${altResponse.status}`);

      // Process the alternative response
      let altData;
      try {
        altData = await altResponse.json();
      } catch (parseError) {
        const textResponse = await altResponse.text();
        console.log('Alternative endpoint non-JSON response:', textResponse);
        return res.status(altResponse.status).json({
          error: textResponse,
          status: altResponse.status
        });
      }

      const statusCode = Math.min(Math.max(altResponse.status, 100), 599);
      console.log(`Returning response with status: ${statusCode}`);
      return res.status(statusCode).json(altData);
    }
    // If we get a 405, it might be due to the space still not being ready, so try again after a delay
    else if (response.status === 405) {
      console.log('Received 405 error, attempting retry after additional warm-up...');

      // Wait a bit more and try again
      await new Promise(resolve => setTimeout(resolve, 3000));

      const retryTimeoutPromise = new Promise((_, reject) => {
        setTimeout(() => reject(new Error('Retry request timeout after 30 seconds')), 30000);
      });

      let retryResponse;
      try {
        // Race the fetch request against a timeout
        const retryFetchPromise = fetch(endpointUrl, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'User-Agent': 'Vercel-Proxy/1.0',
            'Connection': 'keep-alive',
            'Accept-Encoding': 'gzip, deflate, br'
          },
          body: JSON.stringify(req.body),
        });

        retryResponse = await Promise.race([retryFetchPromise, retryTimeoutPromise]);
      } catch (networkError) {
        console.error('Network error when calling backend (retry):', networkError.message);
        if (networkError.message.includes('timeout')) {
          return res.status(408).json({
            error: 'Request timeout',
            details: 'Backend service took too long to respond (retry)'
          });
        }
        return res.status(502).json({
          error: 'Network error connecting to backend service (retry)',
          details: networkError.message
        });
      }

      console.log(`Retry response status: ${retryResponse.status}`);

      if (retryResponse.status === 405) {
        // If still getting 405, try alternative endpoints
        console.log('Still receiving 405 error, trying alternative approaches...');

        // Try the /chat endpoint (might be served without /api/v1 prefix in some deployments)
        const altEndpointUrl = `${backendUrl}/chat`;
        console.log(`Trying alternative endpoint: ${altEndpointUrl}`);

        const finalAltTimeoutPromise = new Promise((_, reject) => {
          setTimeout(() => reject(new Error('Final alternative request timeout after 30 seconds')), 30000);
        });

        let altResponse;
        try {
          // Race the fetch request against a timeout
          const finalAltFetchPromise = fetch(altEndpointUrl, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
              'Accept': 'application/json',
              'User-Agent': 'Vercel-Proxy/1.0'
            },
            body: JSON.stringify(req.body),
          });

          altResponse = await Promise.race([finalAltFetchPromise, finalAltTimeoutPromise]);
        } catch (networkError) {
          console.error('Network error when calling alternative endpoint:', networkError.message);
          if (networkError.message.includes('timeout')) {
            return res.status(408).json({
              error: 'Request timeout',
              details: 'Alternative backend service took too long to respond'
            });
          }
          return res.status(502).json({
            error: 'Network error connecting to alternative endpoint',
            details: networkError.message
          });
        }

        console.log(`Alternative endpoint response status: ${altResponse.status}`);

        // Process the alternative response
        let altData;
        try {
          altData = await altResponse.json();
        } catch (parseError) {
          const textResponse = await altResponse.text();
          console.log('Alternative endpoint non-JSON response:', textResponse);
          return res.status(altResponse.status).json({
            error: textResponse,
            status: altResponse.status
          });
        }

        const statusCode = Math.min(Math.max(altResponse.status, 100), 599);
        console.log(`Returning response with status: ${statusCode} from alternative endpoint`);
        return res.status(statusCode).json(altData);
      } else {
        // If retry worked, process normally
        let retryData;
        try {
          retryData = await retryResponse.json();
        } catch (parseError) {
          const textResponse = await retryResponse.text();
          console.log('Retry non-JSON response received:', textResponse);
          return res.status(retryResponse.status).json({
            error: textResponse,
            status: retryResponse.status
          });
        }

        const statusCode = Math.min(Math.max(retryResponse.status, 100), 599);
        console.log(`Returning response with status: ${statusCode} from retry`);
        return res.status(statusCode).json(retryData);
      }
    }
    // Handle other error statuses
    else if (response.status >= 400) {
      console.log(`Received error status ${response.status}, processing response...`);
      let errorData;
      try {
        errorData = await response.json();
      } catch (parseError) {
        const textResponse = await response.text();
        console.log(`Error response (non-JSON): ${textResponse}`);
        return res.status(response.status).json({
          error: textResponse,
          status: response.status
        });
      }

      console.log(`Error response data:`, errorData);
      return res.status(response.status).json(errorData);
    }

    // Process the original response
    let data;
    try {
      data = await response.json();
    } catch (parseError) {
      // If response is not JSON (e.g., plain text error), return as is
      const textResponse = await response.text();
      console.log('Non-JSON response received:', textResponse);
      return res.status(response.status).json({
        error: textResponse,
        status: response.status
      });
    }

    // Return the response from the backend
    // Use Math.min to ensure we don't return an invalid status code
    const statusCode = Math.min(Math.max(response.status, 100), 599);
    console.log(`Returning successful response with status: ${statusCode}`);
    res.status(statusCode).json(data);
  } catch (error) {
    console.error('Proxy error:', error);
    console.error('Error stack:', error.stack);
    res.status(500).json({
      error: 'Internal server error',
      details: error.message,
      stack: process.env.NODE_ENV === 'development' ? error.stack : undefined
    });
  }
}