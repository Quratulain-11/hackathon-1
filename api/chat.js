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
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Get the backend URL from environment variables
    const backendUrl = process.env.BACKEND_URL || process.env.REACT_APP_BACKEND_URL || process.env.NEXT_PUBLIC_BACKEND_URL || 'https://nainee-chatbot.hf.space';

    console.log('Backend URL:', backendUrl);
    console.log('Request body:', req.body);

    // Try multiple endpoint patterns with comprehensive error handling
    const endpointsToTry = [
      `${backendUrl}/api/v1/chat`,  // Standard documented endpoint
      `${backendUrl}/chat`,         // Simplified endpoint
      `${backendUrl}/query`,        // Alternative common endpoint
      `${backendUrl}/ask`,          // Another common endpoint
    ];

    let response;
    let success = false;
    let lastError = null;

    // Try each endpoint until one succeeds
    for (const endpointUrl of endpointsToTry) {
      try {
        console.log(`Attempting to call: ${endpointUrl}`);

        response = await fetch(endpointUrl, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'User-Agent': 'Vercel-Proxy/1.0'
          },
          body: JSON.stringify(req.body),
        });

        console.log(`Response status from ${endpointUrl}: ${response.status}`);

        // If we get a successful response (2xx), or a 429 (rate limit, which is expected), break the loop
        if (response.status >= 200 && response.status < 300) {
          success = true;
          break;
        }

        // For 429 (rate limit), also consider it a success since it means the endpoint exists
        if (response.status === 429) {
          console.log('Rate limit response received - endpoint is accessible');
          success = true;
          break;
        }

        // Log non-successful responses for debugging
        const errorText = await response.text();
        console.log(`Endpoint ${endpointUrl} returned status ${response.status}: ${errorText}`);

      } catch (fetchError) {
        console.error(`Network error when calling ${endpointUrl}:`, fetchError.message);
        lastError = fetchError;
        continue; // Try the next endpoint
      }
    }

    if (!success) {
      // If all endpoints failed, return an error
      return res.status(502).json({
        error: 'Unable to connect to backend service',
        details: lastError ? lastError.message : 'All endpoint attempts failed'
      });
    }

    // Get the response from the backend
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
    res.status(statusCode).json(data);
  } catch (error) {
    console.error('Proxy error:', error);
    res.status(500).json({
      error: 'Internal server error',
      details: error.message
    });
  }
}