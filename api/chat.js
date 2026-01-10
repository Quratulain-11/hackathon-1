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

    // First, check if the Hugging Face Space is accessible at all
    try {
      console.log('Checking Hugging Face Space availability...');
      const checkResponse = await fetch(backendUrl, {
        method: 'GET',
        headers: {
          'User-Agent': 'Vercel-Proxy/1.0'
        }
      });
      console.log(`Space availability check status: ${checkResponse.status}`);
    } catch (checkError) {
      console.error('Failed to reach Hugging Face Space:', checkError.message);
      return res.status(502).json({
        error: 'Unable to reach backend service',
        details: checkError.message
      });
    }

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

    // If we get a 405, try to determine if it's a method issue or if the endpoint exists differently
    if (response.status === 405) {
      console.log('Received 405 error, trying alternative approach...');

      // Try the /chat endpoint directly (without the /api/v1 prefix)
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