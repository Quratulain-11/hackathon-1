# Vercel Deployment Guide

This document provides instructions for deploying the Docusaurus documentation site with the integrated chatbot to Vercel.

## Overview

The site is configured for deployment to Vercel with the following features:
- Docusaurus static site generation
- Integrated chatbot component that appears on all pages
- Vercel API route proxy to handle backend communication and avoid CORS issues

## Configuration

### Environment Variables

Set the following environment variables in your Vercel project settings:

#### Required Variables
- `BACKEND_URL` or `REACT_APP_BACKEND_URL` or `NEXT_PUBLIC_BACKEND_URL`: The URL of your backend API
  - Example: `https://your-backend-deployment-url.com`
  - Default: `https://nainee-chatbot.hf.space`

#### Optional Variables (Recommended for Production)
- `ALLOWED_ORIGIN` or `NEXT_PUBLIC_ALLOWED_ORIGIN`: The origin allowed to make requests to the API
  - When deployed on Vercel, this will automatically be set to your Vercel project URL
  - Example: `https://your-project-name.vercel.app`

### Deployment Configuration

The `vercel.json` file is configured to:
1. Build the Docusaurus site from the `docs/` directory
2. Handle API routes from the `api/` directory
3. Serve static files from the built site

## Deployment Steps

1. **Connect your GitHub repository** to Vercel
2. **Set environment variables** in the Vercel dashboard:
   - Go to your project dashboard
   - Navigate to Settings → Environment Variables
   - Add the required variables with their values
3. **Configure build settings** (if needed):
   - Build Command: `cd docs && npm ci && npm run build`
   - Output Directory: `docs/build`
   - Install Command: `npm install`

## API Route Proxy

The `/api/chat` route serves as a proxy to communicate with the backend service, avoiding CORS issues when deployed to Vercel. The frontend automatically detects the deployment environment and uses the proxy when appropriate.

## Environment Variable Priority

The application uses the following priority order for backend URL configuration:
1. `NEXT_PUBLIC_BACKEND_URL`
2. `REACT_APP_BACKEND_URL`
3. `BACKEND_URL`
4. Default fallback: `https://nainee-chatbot.hf.space`

## Troubleshooting

### Common Issues

- **CORS errors**: Ensure the API route proxy is working correctly and environment variables are properly set
- **Backend URL not configured**: Set the `BACKEND_URL`, `REACT_APP_BACKEND_URL`, or `NEXT_PUBLIC_BACKEND_URL` environment variable in Vercel dashboard
- **Chatbot not appearing**: Verify the Layout override is properly implemented in `docs/src/theme/Layout/index.js`
- **Connection errors**: Check that your backend URL is accessible and properly configured

### Verification

After deployment:
1. Navigate to your deployed site
2. Verify the chatbot appears on all pages
3. Test sending a message to ensure the backend communication works
4. Check browser developer tools for any console errors related to API calls