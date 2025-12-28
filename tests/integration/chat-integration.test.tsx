/**
 * Integration tests for frontend-backend communication
 */
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import ChatWidget from '../docs/src/components/Chatbot/ChatWidget';

// Mock the API client
jest.mock('../docs/src/components/Chatbot/api/client', () => ({
  sendMessage: jest.fn(() => Promise.resolve({ response: 'Test response from backend' })),
}));

describe('Chat Integration', () => {
  test('full chat flow: user input -> API call -> response display', async () => {
    const { sendMessage } = require('../docs/src/components/Chatbot/api/client');

    render(<ChatWidget />);

    // Initially, chat panel is closed
    expect(screen.queryByRole('form')).not.toBeInTheDocument();

    // Open the chat
    fireEvent.click(screen.getByLabelText(/Open chat/i));

    // Verify chat panel is now open
    expect(screen.getByRole('form')).toBeInTheDocument();

    // Find the input field and submit button
    const input = screen.getByPlaceholderText(/Ask a question about the documentation/i);
    const submitButton = screen.getByRole('button', { name: /send/i });

    // Type a message
    fireEvent.change(input, { target: { value: 'What is this documentation about?' } });

    // Submit the message
    fireEvent.click(submitButton);

    // Verify the API was called with the correct message
    await waitFor(() => {
      expect(sendMessage).toHaveBeenCalledWith('What is this documentation about?');
    });

    // Verify the user message appears in the chat
    expect(screen.getByText('What is this documentation about?')).toBeInTheDocument();

    // Simulate the response appearing after API call
    await waitFor(() => {
      expect(screen.getByText('Test response from backend')).toBeInTheDocument();
    });
  });

  test('handles API errors gracefully', async () => {
    // Mock an API error
    const { sendMessage } = require('../docs/src/components/Chatbot/api/client');
    (sendMessage as jest.Mock).mockRejectedValue(new Error('API Error'));

    render(<ChatWidget />);

    // Open the chat
    fireEvent.click(screen.getByLabelText(/Open chat/i));

    const input = screen.getByPlaceholderText(/Ask a question about the documentation/i);
    const submitButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(input, { target: { value: 'Test error message' } });
    fireEvent.click(submitButton);

    // Wait for error handling
    await waitFor(() => {
      expect(sendMessage).toHaveBeenCalledWith('Test error message');
    });

    // Verify error message appears in the chat
    expect(screen.getByText('API Error')).toBeInTheDocument();
  });

  test('loading state is shown during API calls', async () => {
    // Create a promise that doesn't resolve immediately to test loading state
    const { sendMessage } = require('../docs/src/components/Chatbot/api/client');
    (sendMessage as jest.Mock).mockImplementation(() =>
      new Promise(resolve => setTimeout(() => resolve({ response: 'Delayed response' }), 100))
    );

    render(<ChatWidget />);

    // Open the chat
    fireEvent.click(screen.getByLabelText(/Open chat/i));

    const input = screen.getByPlaceholderText(/Ask a question about the documentation/i);
    const submitButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(input, { target: { value: 'Test loading' } });
    fireEvent.click(submitButton);

    // Button should be disabled (loading state)
    expect(submitButton).toBeDisabled();

    // Wait for the call to complete
    await waitFor(() => {
      expect(screen.getByText('Delayed response')).toBeInTheDocument();
    });
  });
});