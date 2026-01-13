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

    // Prepare the request body in the correct Hugging Face format
    const userMessage = req.body.query || req.body.message || req.body.input || JSON.stringify(req.body);
    const hfRequestBody = {
      data: [userMessage]
    };

    // Use the correct Hugging Face endpoint for predictions
    const endpointUrl = `${backendUrl}/run/predict`;
    console.log(`Attempting to call: ${endpointUrl}`);
    console.log(`Sending request body:`, hfRequestBody);

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
        return res.status(200).json({
          answer: "I'm temporarily unable to process your request. Please try again in a moment.",
          sources: [],
          query: req.body.query || req.body.message || '',
          message: "The backend service is experiencing issues. Our team has been notified."
        });
      }
      return res.status(200).json({
        answer: "I'm temporarily unable to process your request. Please try again in a moment.",
        sources: [],
        query: req.body.query || req.body.message || '',
        message: "The backend service is experiencing issues. Our team has been notified."
      });
    }

    console.log(`Response status: ${response.status}`);

    // Handle 404 - the /run/predict endpoint might not exist, try alternatives
    if (response.status === 404) {
      console.log('Received 404 error, trying alternative endpoints...');

      // Try /api/predict endpoint as alternative
      const altEndpointUrl = `${backendUrl}/api/predict`;
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
          body: JSON.stringify(hfRequestBody),
        });

        altResponse = await Promise.race([altFetchPromise, altTimeoutPromise]);
      } catch (networkError) {
        console.error('Network error when calling alternative endpoint:', networkError.message);
        return res.status(200).json({
          answer: "I'm temporarily unable to process your request. Please try again in a moment.",
          sources: [],
          query: req.body.query || req.body.message || '',
          message: "The backend service is experiencing issues. Our team has been notified."
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
        return res.status(200).json({
          answer: "I'm temporarily unable to process your request. Please try again in a moment.",
          sources: [],
          query: req.body.query || req.body.message || '',
          message: "The backend service is experiencing issues. Our team has been notified."
        });
      }

      // Extract the result from the response
      let answer = '';
      if (altData && Array.isArray(altData.data)) {
        answer = altData.data[0] || JSON.stringify(altData);
      } else if (altData && typeof altData === 'object') {
        // Try to find the answer in different possible fields
        answer = altData.answer || altData.response || altData.result || altData.generated_text || JSON.stringify(altData);
      } else {
        answer = String(altData);
      }

      const statusCode = Math.min(Math.max(altResponse.status, 100), 599);
      console.log(`Returning response with status: ${statusCode}`);
      return res.status(statusCode).json({
        answer: answer,
        sources: [],
        query: req.body.query || req.body.message || ''
      });
    }
    // Handle 405 - Method not allowed, try again after delay (common with cold starts)
    else if (response.status === 405) {
      console.log('Received 405 error, attempting retry after delay...');

      // Wait a bit and try again (for cold start issues)
      await new Promise(resolve => setTimeout(resolve, 3000));

      try {
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
          // If retry worked, process normally
          let retryData;
          try {
            retryData = await retryResponse.json();
          } catch (parseError) {
            const textResponse = await retryResponse.text();
            console.log('Retry non-JSON response received:', textResponse);
            return res.status(200).json({
              answer: "I'm temporarily unable to process your request. Please try again in a moment.",
              sources: [],
              query: req.body.query || req.body.message || '',
              message: "The backend service is experiencing issues. Our team has been notified."
            });
          }

          // Extract the result from the response
          let answer = '';
          if (retryData && Array.isArray(retryData.data)) {
            answer = retryData.data[0] || JSON.stringify(retryData);
          } else if (retryData && typeof retryData === 'object') {
            // Try to find the answer in different possible fields
            answer = retryData.answer || retryData.response || retryData.result || retryData.generated_text || JSON.stringify(retryData);
          } else {
            answer = String(retryData);
          }

          const statusCode = Math.min(Math.max(retryResponse.status, 100), 599);
          console.log(`Returning response with status: ${statusCode} from retry`);
          return res.status(statusCode).json({
            answer: answer,
            sources: [],
            query: req.body.query || req.body.message || ''
          });
        } else {
          // If still getting 405, return friendly message
          return res.status(200).json({
            answer: "I'm temporarily unable to process your request. Please try again in a moment.",
            sources: [],
            query: req.body.query || req.body.message || '',
            message: "The backend service is experiencing issues. Our team has been notified."
          });
        }
      } catch (retryError) {
        console.error('Retry failed:', retryError.message);
        return res.status(200).json({
          answer: "I'm temporarily unable to process your request. Please try again in a moment.",
          sources: [],
          query: req.body.query || req.body.message || '',
          message: "The backend service is experiencing issues. Our team has been notified."
        });
      }
    }
    // Handle other error statuses
    else if (response.status >= 400) {
      console.log(`Received error status ${response.status}, attempting retry...`);

      // Try once more after a short delay (for temporary backend issues)
      await new Promise(resolve => setTimeout(resolve, 2000));

      try {
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

        if (retryResponse.status < 400) {
          // If retry worked, process normally
          let retryData;
          try {
            retryData = await retryResponse.json();
          } catch (parseError) {
            const textResponse = await retryResponse.text();
            console.log('Retry non-JSON response received:', textResponse);
            return res.status(200).json({
              answer: "I'm temporarily unable to process your request. Please try again in a moment.",
              sources: [],
              query: req.body.query || req.body.message || '',
              message: "The backend service is experiencing issues. Our team has been notified."
            });
          }

          // Extract the result from the response
          let answer = '';
          if (retryData && Array.isArray(retryData.data)) {
            answer = retryData.data[0] || JSON.stringify(retryData);
          } else if (retryData && typeof retryData === 'object') {
            // Try to find the answer in different possible fields
            answer = retryData.answer || retryData.response || retryData.result || retryData.generated_text || JSON.stringify(retryData);
          } else {
            answer = String(retryData);
          }

          const statusCode = Math.min(Math.max(retryResponse.status, 100), 599);
          console.log(`Returning response with status: ${statusCode} from retry`);
          return res.status(statusCode).json({
            answer: answer,
            sources: [],
            query: req.body.query || req.body.message || ''
          });
        } else {
          // If retry also failed, return friendly message
          return res.status(200).json({
            answer: "I'm temporarily unable to process your request. Please try again in a moment.",
            sources: [],
            query: req.body.query || req.body.message || '',
            message: "The backend service is experiencing issues. Our team has been notified."
          });
        }
      } catch (retryError) {
        console.error('Retry failed:', retryError.message);
        return res.status(200).json({
          answer: "I'm temporarily unable to process your request. Please try again in a moment.",
          sources: [],
          query: req.body.query || req.body.message || '',
          message: "The backend service is experiencing issues. Our team has been notified."
        });
      }
    }

    // Process the successful response
    let data;
    try {
      data = await response.json();
    } catch (parseError) {
      // If response is not JSON (e.g., plain text error), return friendly message
      const textResponse = await response.text();
      console.log('Non-JSON response received:', textResponse);
      return res.status(200).json({
        answer: "I'm temporarily unable to process your request. Please try again in a moment.",
        sources: [],
        query: req.body.query || req.body.message || '',
        message: "The backend service is experiencing issues. Our team has been notified."
      });
    }

    // Extract the result from the response (return result.data[0] as specified)
    let answer = '';
    if (data && Array.isArray(data.data)) {
      answer = data.data[0] || JSON.stringify(data);
    } else if (data && typeof data === 'object') {
      // Try to find the answer in different possible fields
      answer = data.answer || data.response || data.result || data.generated_text || JSON.stringify(data);
    } else {
      answer = String(data);
    }

    // Return the response from the backend
    // Use Math.min to ensure we don't return an invalid status code
    const statusCode = Math.min(Math.max(response.status, 100), 599);
    console.log(`Returning successful response with status: ${statusCode}`);
    res.status(statusCode).json({
      answer: answer,
      sources: [],
      query: req.body.query || req.body.message || ''
    });
  } catch (error) {
    console.error('Proxy error:', error);
    console.error('Error stack:', error.stack);

    // Return a friendly error message instead of propagating the error
    res.status(200).json({
      error: 'Backend service temporarily unavailable',
      details: 'Please try again in a moment',
      answer: "I'm currently unable to process your request. Please try again in a moment.",
      sources: [],
      query: req.body?.query || req.body?.message || '',
      message: 'The backend service is experiencing issues. Our team has been notified.'
    });
  }
}