/**
 * Custom Layout component with safe chatbot integration
 */
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { useDocusaurusContext } from '@docusaurus/core/hooks';
import { initializeBackendConfig } from '@site/src/utils/backendConfig';
import SafeChatbot from '../../components/Chatbot/SafeChatbot';

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
      <SafeChatbot />
    </>
  );
}