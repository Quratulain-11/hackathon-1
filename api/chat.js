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

    // Forward the request to the backend
    // Forward the request to the backend using the correct endpoint
    // Based on the OpenAPI specification from the Hugging Face Space
    const response = await fetch(`${backendUrl}/api/v1/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(req.body),
    });

    // Get the response from the backend
    const data = await response.json();

    // Return the response from the backend
    // Use Math.min to ensure we don't return an invalid status code
    const statusCode = Math.min(Math.max(response.status, 100), 599);
    res.status(statusCode).json(data);
  } catch (error) {
    console.error('Proxy error:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
}