# Vercel Environment Variables Configuration

This document outlines the environment variables needed for proper Vercel deployment of the frontend-backend integration.

## Required Environment Variables

### For Backend Connection
- `BACKEND_URL` (or `REACT_APP_BACKEND_URL` or `NEXT_PUBLIC_BACKEND_URL`): The URL of your deployed backend API
  - Example: `https://your-backend-deployment-url.com`
  - Default: `https://nainee-chatbot.hf.space`

### For CORS Configuration (Optional but Recommended)
- `ALLOWED_ORIGIN` (or `NEXT_PUBLIC_ALLOWED_ORIGIN`): The origin allowed to make requests to the API
  - When deployed on Vercel, this will automatically be set to `https://your-project-name.vercel.app`
  - For local development, this can be `http://localhost:3000`

## Setting Environment Variables in Vercel

### Using Vercel Dashboard
1. Go to your Vercel project dashboard
2. Navigate to Settings → Environment Variables
3. Add the required variables with their values

### Using Vercel CLI
```bash
vercel env add BACKEND_URL production
# Then enter your backend URL when prompted
```

## Example Values
```
BACKEND_URL=https://your-actual-backend-url.com
# or
REACT_APP_BACKEND_URL=https://your-actual-backend-url.com
# or
NEXT_PUBLIC_BACKEND_URL=https://your-actual-backend-url.com
```

## Deployment Notes
- The API route at `/api/chat` acts as a proxy to avoid CORS issues
- The frontend will automatically use the proxy when deployed to Vercel
- Local development will still connect directly to the backend URL