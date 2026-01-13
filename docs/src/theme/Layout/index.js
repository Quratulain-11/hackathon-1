/**
 * Custom Layout component with safe chatbot integration
 */
import React, { Suspense } from 'react';
import OriginalLayout from '@theme-original/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { initializeBackendConfig } from '@site/src/utils/backendConfig';

// Dynamically import the chatbot component to avoid SSR issues
const Chatbot = React.lazy(() => import('../../components/Chatbot'));

export default function Layout(props) {
  const { siteConfig } = useDocusaurusContext();

  // Initialize backend config with values from docusaurus.config.js
  React.useEffect(() => {
    if (siteConfig && siteConfig.customFields) {
      initializeBackendConfig(siteConfig.customFields);
    }
  }, [siteConfig]);

  // Inject backend config script
  React.useEffect(() => {
    if (siteConfig && siteConfig.customFields && typeof window !== 'undefined') {
      window.__BACKEND_CONFIG__ = {
        baseUrl: siteConfig.customFields.backendUrl || 'https://nainee-chatbot.hf.space'
      };
    }
  }, [siteConfig]);

  return (
    <>
      <OriginalLayout {...props} />
      <Suspense fallback={null}>
        <Chatbot />
      </Suspense>
    </>
  );
}