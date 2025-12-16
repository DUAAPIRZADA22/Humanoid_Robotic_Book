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

  // Create API client from Docusaurus config with error handling
  const apiClient = createApiClient(
    chatApiEndpoint || '',
    chatApiKey || ''
  );

  /**
   * Handle streaming response chunks
   */
  const handleStreamChunk = useCallback((chunk: any) => {
    // Validate chunk structure
    if (!chunk || typeof chunk !== 'object') {
      console.warn('Invalid streaming chunk received:', chunk);
      return;
    }

    try {
      switch (chunk.type) {
        case 'metadata':
          // Start streaming with new message ID
          dispatch(chatActions.startStream(`msg_${Date.now()}`));
          break;

        case 'content':
          // Append content to streaming message
          const content = chunk.content || '';
          if (typeof content === 'string') {
            dispatch(chatActions.appendStream(content + '\n\n'));
          } else {
            console.warn('Invalid content in chunk:', chunk);
          }
          break;

        case 'complete':
          // End streaming
          dispatch(chatActions.endStream());
          break;

        case 'error':
          // Handle streaming error
          dispatch(chatActions.setError({
            code: 'STREAM_ERROR',
            message: chunk.message || chunk.error || 'Streaming error occurred',
            retryable: true,
            timestamp: new Date(),
          }));
          dispatch(chatActions.endStream());
          break;

        default:
          console.warn('Unknown chunk type:', chunk.type);
      }
    } catch (error) {
      console.error('Error handling stream chunk:', error);
      dispatch(chatActions.setError({
        code: 'CHUNK_PROCESSING_ERROR',
        message: 'Failed to process response chunk',
        retryable: true,
        timestamp: new Date(),
      }));
      dispatch(chatActions.endStream());
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
    // Validate input parameters
    if (!content || typeof content !== 'string') {
      console.error('Invalid content provided to sendStreamMessage:', content);
      return;
    }

    console.log('Sending message to API:', { content, metadata, endpoint: chatApiEndpoint });

    // Check if API is configured (use mock responses only if no endpoint)
    if (!chatApiEndpoint) {
      console.log('No API endpoint configured, using mock response');
      try {
        // Create a mock response when no backend is available
        dispatch(chatActions.setLoading(true));

        // Simulate API delay
        await new Promise(resolve => setTimeout(resolve, 1000));

        // Create mock bot response
        const mockResponses = [
          "I'm a demo chatbot! The backend API is not currently running, so I can't provide intelligent responses. To enable full functionality, please start the chatbot backend server or configure the API endpoints.",
          "Hello! This is a demonstration of the chat widget interface. To get actual AI responses, you'll need to run the backend chatbot service that connects to your AI model.",
          "Thanks for your message! The chatbot backend is currently offline. You can start it by running the Python chatbot server from the project's backend directory, or configure the API endpoints in your environment variables."
        ];

        const mockResponse = mockResponses[Math.floor(Math.random() * mockResponses.length)];

        // Add bot response using correct message structure
        dispatch(chatActions.addMessage({
          id: `bot_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
          content: mockResponse,
          role: 'assistant',
          timestamp: new Date(),
        }));

        dispatch(chatActions.setLoading(false));
      } catch (error) {
        console.error('Error in mock response generation:', error);
        dispatch(chatActions.setError({
          code: 'MOCK_RESPONSE_ERROR',
          message: 'Failed to generate response',
          retryable: true,
          timestamp: new Date(),
        }));
        dispatch(chatActions.setLoading(false));
      }
      return;
    }

    // Set loading state
    dispatch(chatActions.setLoading(true));

    // Clear any previous cleanup
    if (cleanupRef.current) {
      cleanupRef.current();
      cleanupRef.current = null;
    }

    // Prepare request payload to match backend expectations
    const request = {
      query: content,
      top_k: 5,
      similarity_threshold: 0.7,
      use_rerank: true,
    };

    console.log('Starting API call with request:', request);
    console.log('API Client config:', { endpoint: apiClient.endpoint, hasApiKey: !!apiClient.apiKey });

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
    const handleSendMessage = (event: Event) => {
      try {
        const customEvent = event as CustomEvent;
        if (!customEvent.detail) {
          console.error('Received send message event without detail:', event);
          return;
        }

        const { content, metadata } = customEvent.detail;
        sendStreamMessage(content, metadata);
      } catch (error) {
        console.error('Error handling send message event:', error);
        dispatch(chatActions.setError({
          code: 'EVENT_HANDLING_ERROR',
          message: 'Failed to handle message send event',
          retryable: true,
          timestamp: new Date(),
        }));
      }
    };

    // Add event listener
    window.addEventListener('chat:send-message', handleSendMessage);

    // Cleanup on unmount
    return () => {
      window.removeEventListener('chat:send-message', handleSendMessage);
      if (cleanupRef.current) {
        cleanupRef.current();
      }
    };
  }, [sendStreamMessage, dispatch]);

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