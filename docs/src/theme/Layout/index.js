/**
 * Custom Layout component with safe chatbot integration
 */
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import SafeChatbot from '@site/src/components/Chatbot/SafeChatbot';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <SafeChatbot />
    </>
  );
}