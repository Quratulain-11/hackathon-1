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

    // Try the documented endpoint first: /api/v1/chat
    let response;
    let endpointUrl = `${backendUrl}/api/v1/chat`;
    let requestHeaders = {
      'Content-Type': 'application/json',
      'Accept': 'application/json',
      'User-Agent': 'Vercel-Proxy/1.0'
    };

    try {
      response = await fetch(endpointUrl, {
        method: 'POST',
        headers: requestHeaders,
        body: JSON.stringify(req.body),
      });
    } catch (fetchError) {
      console.error(`Failed to reach backend at ${endpointUrl}:`, fetchError.message);
      // If the first endpoint fails completely, try the fallback
      endpointUrl = `${backendUrl}/chat`;
      response = await fetch(endpointUrl, {
        method: 'POST',
        headers: requestHeaders,
        body: JSON.stringify(req.body),
      });
    }

    // If we get a 404 "Not Found" error or 405 "Method Not Allowed", try the root-level chat endpoint
    // This handles cases where Hugging Face Spaces expose APIs differently
    if (response.status === 404 || response.status === 405) {
      const errorText = await response.text();
      console.log(`Backend returned ${response.status}: ${errorText}`);
      if (errorText.includes('Not Found') || response.status === 404 || response.status === 405) {
        endpointUrl = `${backendUrl}/chat`;
        response = await fetch(endpointUrl, {
          method: 'POST',
          headers: requestHeaders,
          body: JSON.stringify(req.body),
        });
      }
    }

    // Get the response from the backend
    let data;
    try {
      data = await response.json();
    } catch (parseError) {
      // If response is not JSON (e.g., plain text error), return as is
      const textResponse = await response.text();
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