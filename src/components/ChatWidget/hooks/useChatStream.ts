/**
 * Hook for handling chat streaming via Server-Sent Events
 * @fileoverview Custom hook for managing SSE connections and message streaming
 */

import { useCallback, useRef, useEffect } from 'react';
import { useChatContext } from '../contexts/ChatContext';
import { createApiClient, streamChatMessage } from '../utils/api';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { chatActions } from '../contexts/chatReducer';

/**
 * Return type for useChatStream hook
 */
interface UseChatStreamReturn {
  /** Whether currently streaming */
  isStreaming: boolean;
  /** Current error if any */
  error: any;
  /** Send a message to the API */
  sendStreamMessage: (content: string, metadata?: any) => Promise<void>;
}

/**
 * Hook for managing chat streaming functionality
 * @returns Chat streaming utilities
 */
export function useChatStream(): UseChatStreamReturn {
  const { state, dispatch } = useChatContext();
  const { siteConfig } = useDocusaurusContext();
  const cleanupRef = useRef<(() => void) | null>(null);

  // Check if API configuration is available
  const chatApiEndpoint = siteConfig?.customFields?.chatApiEndpoint;
  const chatApiKey = siteConfig?.customFields?.chatApiKey;

  // Create API client from Docusaurus config
  const apiClient = createApiClient(
    chatApiEndpoint,
    chatApiKey
  );

  /**
   * Handle streaming response chunks
   */
  const handleStreamChunk = useCallback((chunk: any) => {
    switch (chunk.type) {
      case 'start':
        // Start streaming with new message ID
        dispatch(chatActions.startStream(chunk.sessionId || `msg_${Date.now()}`));
        break;

      case 'chunk':
        // Append content to streaming message
        dispatch(chatActions.appendStream(chunk.content || ''));
        break;

      case 'error':
        // Handle streaming error
        dispatch(chatActions.setError({
          code: 'STREAM_ERROR',
          message: chunk.error || 'Streaming error occurred',
          retryable: true,
          timestamp: new Date(),
        }));
        dispatch(chatActions.endStream());
        break;

      case 'done':
        // End streaming
        dispatch(chatActions.endStream());
        break;
    }
  }, [dispatch]);

  /**
   * Handle streaming completion
   */
  const handleStreamComplete = useCallback(() => {
    // Cleanup will be handled by the cleanup function
  }, []);

  /**
   * Handle streaming errors
   */
  const handleStreamError = useCallback((error: any) => {
    dispatch(chatActions.setError(error));
  }, [dispatch]);

  /**
   * Send a message with streaming support
   */
  const sendStreamMessage = useCallback(async (
    content: string,
    metadata?: any
  ): Promise<void> => {
    // Check if API is configured
    if (!chatApiEndpoint || !chatApiKey || chatApiEndpoint.includes('localhost:7860')) {
      // Create a mock response when no backend is available
      dispatch(chatActions.setLoading(true));

      // Simulate API delay
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Add user message
      dispatch(chatActions.addMessage({
        id: `user_${Date.now()}`,
        content,
        type: 'user',
        timestamp: new Date(),
      }));

      // Create mock bot response
      const mockResponses = [
        "I'm a demo chatbot! The backend API is not currently running, so I can't provide intelligent responses. To enable full functionality, please start the chatbot backend server or configure the API endpoints.",
        "Hello! This is a demonstration of the chat widget interface. To get actual AI responses, you'll need to run the backend chatbot service that connects to your AI model.",
        "Thanks for your message! The chatbot backend is currently offline. You can start it by running the Python chatbot server from the project's backend directory, or configure the API endpoints in your environment variables."
      ];

      const mockResponse = mockResponses[Math.floor(Math.random() * mockResponses.length)];

      // Add bot response
      dispatch(chatActions.addMessage({
        id: `bot_${Date.now()}`,
        content: mockResponse,
        type: 'bot',
        timestamp: new Date(),
      }));

      dispatch(chatActions.setLoading(false));
      return;
    }

    // Set loading state
    dispatch(chatActions.setLoading(true));

    // Clear any previous cleanup
    if (cleanupRef.current) {
      cleanupRef.current();
      cleanupRef.current = null;
    }

    // Prepare request payload
    const request = {
      question: content,
      stream: true,
      context: (metadata?.selectedText && typeof window !== 'undefined') ? {
        selectedText: metadata.selectedText,
        pageUrl: window.location.pathname,
        pageTitle: document.title,
      } : undefined,
    };

    // Start streaming
    const cleanup = streamChatMessage(
      apiClient,
      request,
      handleStreamChunk,
      handleStreamComplete,
      handleStreamError
    );

    // Store cleanup function
    cleanupRef.current = cleanup;
  }, [apiClient, dispatch, handleStreamChunk, handleStreamComplete, handleStreamError]);

  // Listen for send message events from context
  useEffect(() => {
    const handleSendMessage = (event: CustomEvent) => {
      const { content, metadata } = event.detail;
      sendStreamMessage(content, metadata);
    };

    // Add event listener
    window.addEventListener('chat:send-message', handleSendMessage as EventListener);

    // Cleanup on unmount
    return () => {
      window.removeEventListener('chat:send-message', handleSendMessage as EventListener);
      if (cleanupRef.current) {
        cleanupRef.current();
      }
    };
  }, [sendStreamMessage]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (cleanupRef.current) {
        cleanupRef.current();
      }
    };
  }, []);

  return {
    isStreaming: state.isStreaming,
    error: state.error,
    sendStreamMessage,
  };
}