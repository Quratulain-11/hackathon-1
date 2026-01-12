/**
 * Custom Layout component with safe chatbot integration
 */
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { initializeBackendConfig } from '@site/src/utils/backendConfig';
import ChatbotPortal from '../../components/Chatbot/ChatbotPortal';

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
      <ChatbotPortal />
    </>
  );
}