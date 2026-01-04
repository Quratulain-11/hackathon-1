// Vercel API route to proxy requests to the backend
// This helps avoid CORS issues when deployed to Vercel
export default async function handler(req, res) {
  // Only allow POST requests
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Get the backend URL from environment variables
    const backendUrl = process.env.BACKEND_URL || process.env.REACT_APP_BACKEND_URL || 'https://nainee-chatbot.hf.space';

    // Forward the request to the backend
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
    res.status(response.status).json(data);
  } catch (error) {
    console.error('Proxy error:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
}