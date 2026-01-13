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

    // Convert the request body to the Hugging Face format
    // Hugging Face typically expects { "data": [input_values...] } or { "inputs": input_value }
    const hfRequestBody = {
      inputs: req.body.query || req.body.message || req.body.data || JSON.stringify(req.body),
      parameters: {
        top_k: req.body.top_k || 5,
        temperature: req.body.temperature || 0.7,
        max_tokens: req.body.max_tokens || 500
      }
    };

    // Try the Hugging Face prediction endpoint: /api/predict or /run/predict
    let endpointUrl = `${backendUrl}/api/predict`;
    console.log(`Attempting to call: ${endpointUrl}`);

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
        body: JSON.stringify(hfRequestBody),
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

    // Handle 404 - try alternative endpoints for Hugging Face Spaces
    if (response.status === 404) {
      console.log('Received 404 error, trying alternative Hugging Face endpoints...');

      // Try /run/predict endpoint
      endpointUrl = `${backendUrl}/run/predict`;
      console.log(`Trying alternative endpoint: ${endpointUrl}`);

      const altTimeoutPromise = new Promise((_, reject) => {
        setTimeout(() => reject(new Error('Alternative request timeout after 30 seconds')), 30000);
      });

      let altResponse;
      try {
        // Race the fetch request against a timeout
        const altFetchPromise = fetch(endpointUrl, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'User-Agent': 'Vercel-Proxy/1.0'
          },
          body: JSON.stringify(hfRequestBody),
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

      // Format the response to match what the frontend expects
      const formattedResponse = formatHuggingFaceResponse(altData, req.body.query);
      const statusCode = Math.min(Math.max(altResponse.status, 100), 599);
      console.log(`Returning response with status: ${statusCode}`);
      return res.status(statusCode).json(formattedResponse);
    }
    // Handle 405 - Method not allowed, try different approaches
    else if (response.status === 405) {
      console.log('Received 405 error, trying alternative approaches for Hugging Face endpoints...');

      // Try GET method first to see if the endpoint supports it
      try {
        const getResponse = await fetch(`${backendUrl}/api/predict`, {
          method: 'GET',
          headers: {
            'Accept': 'application/json',
            'User-Agent': 'Vercel-Proxy/1.0'
          }
        });

        console.log(`GET method status: ${getResponse.status}`);

        // If GET is allowed, try POST again after a delay (space might be waking up)
        if (getResponse.status !== 405) {
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
            body: JSON.stringify(hfRequestBody),
          });

          console.log(`Retry response status: ${retryResponse.status}`);

          if (retryResponse.status !== 405) {
            // Process normally if not 405
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

            const formattedResponse = formatHuggingFaceResponse(retryData, req.body.query);
            const statusCode = Math.min(Math.max(retryResponse.status, 100), 599);
            console.log(`Returning response with status: ${statusCode} from retry`);
            return res.status(statusCode).json(formattedResponse);
          }
        }
      } catch (getErr) {
        console.log('GET method check failed, continuing with alternatives:', getErr.message);
      }

      // Try /run/predict endpoint as alternative
      const altEndpointUrl = `${backendUrl}/run/predict`;
      console.log(`Trying /run/predict endpoint: ${altEndpointUrl}`);

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
          body: JSON.stringify(hfRequestBody),
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

      const formattedResponse = formatHuggingFaceResponse(altData, req.body.query);
      const statusCode = Math.min(Math.max(altResponse.status, 100), 599);
      console.log(`Returning response with status: ${statusCode} from alternative endpoint`);
      return res.status(statusCode).json(formattedResponse);
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

      // If it's a 405 error, return a friendly message instead of the error
      if (response.status === 405) {
        return res.status(200).json({
          answer: "I'm temporarily unable to process your request. Please try again in a moment.",
          sources: [],
          query: req.body.query,
          message: "The backend service is experiencing issues. Our team has been notified."
        });
      }

      return res.status(response.status).json(errorData);
    }

    // Process the original response from Hugging Face
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

    // Format the Hugging Face response to match frontend expectations
    const formattedResponse = formatHuggingFaceResponse(data, req.body.query);

    // Return the response from the backend
    // Use Math.min to ensure we don't return an invalid status code
    const statusCode = Math.min(Math.max(response.status, 100), 599);
    console.log(`Returning successful response with status: ${statusCode}`);
    res.status(statusCode).json(formattedResponse);
  } catch (error) {
    console.error('Proxy error:', error);
    console.error('Error stack:', error.stack);

    // Return a friendly error message instead of propagating the error
    res.status(500).json({
      error: 'Backend service temporarily unavailable',
      details: 'Please try again in a moment',
      answer: "I'm currently unable to process your request. Please try again in a moment.",
      sources: [],
      query: req.body?.query || '',
      message: 'The backend service is experiencing issues. Our team has been notified.'
    });
  }
}

// Helper function to format Hugging Face response to match frontend expectations
function formatHuggingFaceResponse(hfData, query) {
  // Hugging Face responses can come in different formats
  // Try to extract the answer from various possible structures
  let answer = '';
  let sources = [];

  if (hfData && typeof hfData === 'object') {
    // Common Hugging Face response formats
    if (Array.isArray(hfData)) {
      // If it's an array, take the first element as the answer
      answer = hfData[0] || hfData.join(' ') || 'I processed your request successfully.';
    } else if (hfData.hasOwnProperty('data')) {
      // If it has a 'data' property
      if (Array.isArray(hfData.data)) {
        answer = hfData.data[0] || hfData.data.join(' ');
      } else {
        answer = hfData.data || JSON.stringify(hfData.data);
      }
    } else if (hfData.hasOwnProperty('outputs') || hfData.hasOwnProperty('output')) {
      // If it has 'outputs' or 'output' property
      const output = hfData.outputs || hfData.output;
      if (Array.isArray(output)) {
        answer = output[0] || output.join(' ');
      } else {
        answer = output || JSON.stringify(output);
      }
    } else if (hfData.hasOwnProperty('result')) {
      // If it has a 'result' property
      answer = hfData.result || JSON.stringify(hfData.result);
    } else if (hfData.hasOwnProperty('generated_text')) {
      // Common for text generation models
      answer = hfData.generated_text || JSON.stringify(hfData);
    } else {
      // Try to find any text-like property
      const textProps = ['answer', 'response', 'text', 'content', 'prediction', 'prediction_result'];
      for (const prop of textProps) {
        if (hfData.hasOwnProperty(prop)) {
          answer = hfData[prop];
          break;
        }
      }

      if (!answer) {
        // If no specific property found, stringify the entire object
        answer = JSON.stringify(hfData);
      }
    }

    // Extract sources if available
    if (hfData.hasOwnProperty('sources') || hfData.hasOwnProperty('source_documents')) {
      sources = hfData.sources || hfData.source_documents || [];
    }
  } else {
    // If it's not an object, convert to string
    answer = String(hfData || 'Processing completed.');
  }

  // Ensure answer is a string
  if (typeof answer !== 'string') {
    answer = JSON.stringify(answer);
  }

  return {
    answer: answer,
    sources: sources,
    query: query,
    original_response: hfData // Include original response for debugging if needed
  };
}