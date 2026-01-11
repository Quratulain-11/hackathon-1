// This script is injected by Docusaurus to provide backend configuration to client-side code
// The actual config values are injected server-side via Docusaurus ssr-plugin

// The configuration will be available as window.__BACKEND_CONFIG__
// This is populated by the Docusaurus SSR plugin based on customFields

if (typeof window !== 'undefined' && window.__PRELOADED_DATA__) {
  // Extract backend config from preloaded data if available
  const siteConfig = window.__PRELOADED_DATA__.siteConfig;
  if (siteConfig && siteConfig.customFields && siteConfig.customFields.backendUrl) {
    window.__BACKEND_CONFIG__ = {
      baseUrl: siteConfig.customFields.backendUrl
    };
  }
}

// Fallback: if no config is found, use default
if (typeof window !== 'undefined' && !window.__BACKEND_CONFIG__) {
  window.__BACKEND_CONFIG__ = {
    baseUrl: 'https://nainee-chatbot.hf.space'
  };
}