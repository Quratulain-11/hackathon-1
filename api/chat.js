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

    // Hugging Face Spaces sometimes need to be "warmed up" - they return 405 initially
    // Try to make a preliminary call to "warm up" the space if needed
    const healthCheckUrl = `${backendUrl}/health`;
    try {
      console.log('Warming up Hugging Face Space with health check...');
      await fetch(healthCheckUrl, {
        method: 'GET',
        headers: {
          'User-Agent': 'Vercel-Proxy/1.0'
        }
      });
      // Wait a brief moment for the space to wake up
      await new Promise(resolve => setTimeout(resolve, 1000));
    } catch (healthError) {
      console.log('Health check failed (this is normal for some spaces):', healthError.message);
      // Continue anyway as the health check is just for warming up
    }

    // Now try the main chat endpoint - the documented one
    const endpointUrl = `${backendUrl}/api/v1/chat`;
    console.log(`Attempting to call main endpoint: ${endpointUrl}`);

    const response = await fetch(endpointUrl, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Accept': 'application/json',
        'User-Agent': 'Vercel-Proxy/1.0'
      },
      body: JSON.stringify(req.body),
    });

    console.log(`Response status: ${response.status}`);

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