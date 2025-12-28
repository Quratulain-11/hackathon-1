/**
 * Unit tests for ChatWidget component
 */
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import ChatWidget from './ChatWidget';

// Mock the child components
jest.mock('./ChatButton', () => {
  return {
    __esModule: true,
    default: ({ onClick, isOpen }: { onClick: () => void; isOpen: boolean }) => (
      <button
        data-testid="chat-button"
        onClick={onClick}
        aria-expanded={isOpen}
      >
        {isOpen ? 'Close' : 'Open'} Chat
      </button>
    ),
  };
});

jest.mock('./ChatPanel', () => {
  return {
    __esModule: true,
    default: ({ messages, inputValue, setInputValue, onSubmit, isLoading }: any) => (
      <div data-testid="chat-panel">
        <div data-testid="messages">{messages.length} messages</div>
        <input
          data-testid="chat-input"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          disabled={isLoading}
        />
        <button
          data-testid="submit-button"
          onClick={onSubmit}
          disabled={isLoading}
        >
          Send
        </button>
      </div>
    ),
  };
});

jest.mock('./api/client', () => {
  return {
    sendMessage: jest.fn(() => Promise.resolve({ response: 'Test response' })),
  };
});

describe('ChatWidget', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('renders without crashing', () => {
    render(<ChatWidget />);
    expect(screen.getByTestId('chat-button')).toBeInTheDocument();
  });

  test('toggles chat panel when button is clicked', () => {
    render(<ChatWidget />);

    const button = screen.getByTestId('chat-button');
    expect(button).toBeInTheDocument();

    // Initially, chat panel should not be visible
    expect(screen.queryByTestId('chat-panel')).not.toBeInTheDocument();

    // Click to open
    fireEvent.click(button);
    expect(screen.getByTestId('chat-panel')).toBeInTheDocument();

    // Click to close
    fireEvent.click(button);
    expect(screen.queryByTestId('chat-panel')).not.toBeInTheDocument();
  });

  test('submits a message when form is submitted', async () => {
    const { sendMessage } = require('./api/client');
    render(<ChatWidget />);

    // Open the chat
    fireEvent.click(screen.getByTestId('chat-button'));

    // Find the input and submit button
    const input = screen.getByTestId('chat-input');
    const submitButton = screen.getByTestId('submit-button');

    // Enter a message
    fireEvent.change(input, { target: { value: 'Hello, world!' } });

    // Submit the form
    fireEvent.click(submitButton);

    // Wait for the API call
    await waitFor(() => {
      expect(sendMessage).toHaveBeenCalledWith('Hello, world!');
    });
  });

  test('shows loading state during API call', async () => {
    // Mock a delayed response
    const { sendMessage } = require('./api/client');
    (sendMessage as jest.MockedFunction<any>).mockImplementation(() =>
      new Promise(resolve => setTimeout(() => resolve({ response: 'Test response' }), 100))
    );

    render(<ChatWidget />);

    // Open the chat
    fireEvent.click(screen.getByTestId('chat-button'));

    const input = screen.getByTestId('chat-input');
    const submitButton = screen.getByTestId('submit-button');

    fireEvent.change(input, { target: { value: 'Test message' } });
    fireEvent.click(submitButton);

    // Button should be disabled during loading
    expect(submitButton).toBeDisabled();

    // Wait for the call to complete
    await waitFor(() => {
      expect(sendMessage).toHaveBeenCalledWith('Test message');
    });
  });
});