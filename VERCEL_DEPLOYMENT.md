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

- `BACKEND_URL` or `REACT_APP_BACKEND_URL`: The URL of your backend API (defaults to `https://nainee-chatbot.hf.space`)

### Deployment Configuration

The `vercel.json` file is configured to:
1. Build the Docusaurus site from the `docs/` directory
2. Handle API routes from the `api/` directory
3. Serve static files from the built site

## Deployment Steps

1. **Connect your GitHub repository** to Vercel
2. **Set environment variables** in the Vercel dashboard
3. **Configure build settings** (if needed):
   - Build Command: `cd docs && npm ci && npm run build`
   - Output Directory: `docs/build`
   - Install Command: `npm install`

## API Route Proxy

The `/api/chat` route serves as a proxy to communicate with the backend service, avoiding CORS issues when deployed to Vercel. The frontend automatically detects the deployment environment and uses the proxy when appropriate.

## Troubleshooting

### Common Issues

- **CORS errors**: Ensure the API route proxy is working correctly
- **Backend URL not configured**: Set the `BACKEND_URL` environment variable in Vercel dashboard
- **Chatbot not appearing**: Verify the Layout override is properly implemented

### Verification

After deployment:
1. Navigate to your deployed site
2. Verify the chatbot appears on all pages
3. Test sending a message to ensure the backend communication works