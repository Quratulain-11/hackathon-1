/**
 * Custom Layout component with safe chatbot integration
 */
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { useDocusaurusContext } from '@docusaurus/core';
import ChatbotPortal from '../../components/Chatbot/ChatbotPortal';

export default function Layout(props) {
  const { siteConfig } = useDocusaurusContext();

  // Initialize backend config with values from docusaurus.config.js
  React.useEffect(() => {
    if (siteConfig && siteConfig.customFields) {
      if (siteConfig.customFields.backendUrl && typeof window !== 'undefined') {
        window.__BACKEND_CONFIG__ = {
          baseUrl: siteConfig.customFields.backendUrl
        };
      }
    }
  }, [siteConfig]);

  return (
    <>
      <OriginalLayout {...props} />
      <ChatbotPortal />
    </>
  );
}