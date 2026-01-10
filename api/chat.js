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

    // Function to warm up the Hugging Face Space
    const warmUpSpace = async () => {
      try {
        console.log('Warming up Hugging Face Space...');

        // Try the health endpoint first
        const healthUrl = `${backendUrl}/health`;
        const healthResponse = await fetch(healthUrl, {
          method: 'GET',
          headers: {
            'User-Agent': 'Vercel-Proxy/1.0'
          }
        });
        console.log(`Health check status: ${healthResponse.status}`);

        if (healthResponse.status === 405 || healthResponse.status === 404) {
          // If health endpoint doesn't exist, try the root
          const rootResponse = await fetch(backendUrl, {
            method: 'GET',
            headers: {
              'User-Agent': 'Vercel-Proxy/1.0'
            }
          });
          console.log(`Root check status: ${rootResponse.status}`);
        }

        // Wait a bit for the space to fully wake up
        await new Promise(resolve => setTimeout(resolve, 2000));
      } catch (warmUpError) {
        console.log('Warm-up attempt failed (this is normal for some spaces):', warmUpError.message);
        // Continue anyway as the warm-up is just to help with cold starts
      }
    };

    // Warm up the space before making the main request
    await warmUpSpace();

    // Try the documented endpoint: /api/v1/chat
    const endpointUrl = `${backendUrl}/api/v1/chat`;
    console.log(`Attempting to call: ${endpointUrl}`);

    // Try with additional headers that might be required by Hugging Face
    const response = await fetch(endpointUrl, {
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

    console.log(`Response status: ${response.status}`);

    // If we get a 405, it might be due to the space still not being ready, so try again after a delay
    if (response.status === 405) {
      console.log('Received 405 error, attempting retry after additional warm-up...');

      // Wait a bit more and try again
      await new Promise(resolve => setTimeout(resolve, 3000));

      const retryResponse = await fetch(endpointUrl, {
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

      console.log(`Retry response status: ${retryResponse.status}`);

      if (retryResponse.status === 405) {
        // If still getting 405, try alternative endpoints
        console.log('Still receiving 405 error, trying alternative approaches...');

        // Try the /chat endpoint (might be served without /api/v1 prefix in some deployments)
        const altEndpointUrl = `${backendUrl}/chat`;
        console.log(`Trying alternative endpoint: ${altEndpointUrl}`);

        const altResponse = await fetch(altEndpointUrl, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'User-Agent': 'Vercel-Proxy/1.0'
          },
          body: JSON.stringify(req.body),
        });

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
        return res.status(statusCode).json(retryData);
      }
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
    res.status(statusCode).json(data);
  } catch (error) {
    console.error('Proxy error:', error);
    res.status(500).json({
      error: 'Internal server error',
      details: error.message
    });
  }
}