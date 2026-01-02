/**
 * Hook for handling chat streaming via Server-Sent Events
 * @fileoverview Custom hook for managing SSE connections and message streaming
 */

import { useCallback, useRef, useEffect, useMemo } from 'react';
import { useChatContext } from '../contexts/ChatContext';
import { useAuth } from '../../../contexts/AuthContext';
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
  const { token } = useAuth();

  // Refs to track request state (persistent across re-renders)
  const cleanupRef = useRef<(() => void) | null>(null);
  const isRequestInProgressRef = useRef(false);
  const activeRequestIdRef = useRef<string | null>(null); // Track active request ID

  // Check if API configuration is available
  const chatApiEndpoint = siteConfig?.customFields?.chatApiEndpoint;
  const chatApiKey = siteConfig?.customFields?.chatApiKey;

  // FIX: Memoize apiClient to prevent recreation on every render
  // This was the root cause - new apiClient caused sendStreamMessage to recreate,
  // which caused useEffect to re-run and abort the stream
  const apiClient = useMemo(
    () => createApiClient(chatApiEndpoint || '', chatApiKey || ''),
    [chatApiEndpoint, chatApiKey] // Only recreate when these actually change
  );

  /**
   * Handle streaming response chunks
   * FIX: Skip sources, remove extra formatting for concise answers
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

        case 'source':
          // FIX: Skip source chunks - don't display them
          // Still log for debugging if needed
          console.debug('[SSE] Source skipped:', chunk.source);
          break;

        case 'content':
          // Append content directly without extra newlines
          const content = chunk.content || '';
          if (typeof content === 'string') {
            dispatch(chatActions.appendStream(content));
          } else {
            console.warn('Invalid content in chunk:', chunk);
          }
          break;

        case 'answer_start':
          // FIX: Don't add separator or label, just start answer
          // dispatch(chatActions.appendStream('\n---\n**AI Answer:**\n'));
          break;

        case 'answer_chunk':
          // Append AI-generated answer chunk
          const answerChunk = chunk.content || '';
          if (typeof answerChunk === 'string') {
            dispatch(chatActions.appendStream(answerChunk));
          } else {
            console.warn('Invalid answer chunk:', chunk);
          }
          break;

        case 'complete':
          // FIX: End streaming without trailing separator
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
          console.debug('[SSE] Unknown chunk type:', chunk.type);
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
    // Reset request in progress flag
    isRequestInProgressRef.current = false;
    activeRequestIdRef.current = null; // Clear active request ID
    // Cleanup will be handled by the cleanup function
  }, []);

  /**
   * Handle streaming errors
   */
  const handleStreamError = useCallback((error: any) => {
    // Reset request in progress flag
    isRequestInProgressRef.current = false;
    activeRequestIdRef.current = null; // Clear active request ID
    dispatch(chatActions.setError(error));
  }, [dispatch]);

  /**
   * Send a message with streaming support
   * FIX: Added request ID tracking to prevent aborting active requests
   * when useEffect re-runs due to dependency changes
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

    // Generate unique request ID for this message
    const requestId = `req_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    activeRequestIdRef.current = requestId;

    // Prevent concurrent requests (but allow new request after previous completes)
    if (isRequestInProgressRef.current) {
      console.warn('[Chat] Request already in progress, ignoring duplicate call');
      return;
    }

    console.log('Sending message to API:', { content, metadata, endpoint: chatApiEndpoint, requestId });

    // Check if API is configured (use mock responses only if no endpoint)
    if (!chatApiEndpoint) {
      console.log('No API endpoint configured, using mock response');
      try {
        // Create a mock response when no backend is available
        isRequestInProgressRef.current = true;
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
        isRequestInProgressRef.current = false;
        activeRequestIdRef.current = null;
      } catch (error) {
        console.error('Error in mock response generation:', error);
        dispatch(chatActions.setError({
          code: 'MOCK_RESPONSE_ERROR',
          message: 'Failed to generate response',
          retryable: true,
          timestamp: new Date(),
        }));
        dispatch(chatActions.setLoading(false));
        isRequestInProgressRef.current = false;
        activeRequestIdRef.current = null;
      }
      return;
    }

    // Set loading state and mark request as in progress
    isRequestInProgressRef.current = true;
    dispatch(chatActions.setLoading(true));

    // FIX: Only abort if the cleanup belongs to the CURRENT request
    // Previous cleanup might belong to an already-completed request
    if (cleanupRef.current) {
      // Store the old cleanup function
      const oldCleanup = cleanupRef.current;
      // Clear the ref first
      cleanupRef.current = null;
      // Then call the old cleanup (if it's not for this request)
      // This prevents aborting a new request when useEffect re-runs
      oldCleanup();
    }

    // Prepare request payload to match backend expectations
    const request = {
      question: content,  // Changed from 'query' to 'question' to match backend
      top_k: 3,
      similarity_threshold: 0.2,  // Lower threshold for better retrieval
      use_rerank: false,  // Disabled: reduces API calls from 3+ to 1, prevents timeout
      stream: true,  // Enable streaming for real-time responses
    };

    console.log('Starting API call with request:', request);
    console.log('API Client config:', { endpoint: apiClient.endpoint, hasApiKey: !!apiClient.apiKey });

    // Start streaming
    const cleanup = streamChatMessage(
      apiClient,
      request,
      token,
      handleStreamChunk,
      handleStreamComplete,
      handleStreamError
    );

    // FIX: Only store cleanup if this is still the active request
    // If a new request has started, don't store this cleanup function
    if (activeRequestIdRef.current === requestId) {
      cleanupRef.current = cleanup;
    }
  }, [apiClient, token, dispatch, handleStreamChunk, handleStreamComplete, handleStreamError, chatApiEndpoint]);

  // FIX: Store sendStreamMessage in a ref to avoid re-running useEffect
  // when callback dependencies change (prevents stream abort on re-render)
  const sendStreamMessageRef = useRef(sendStreamMessage);
  sendStreamMessageRef.current = sendStreamMessage;

  // Listen for send message events from context
  // FIX: Use stable event listener pattern - listener only runs once on mount,
  // and uses ref to always call the latest sendStreamMessage implementation
  useEffect(() => {
    const handleSendMessage = (event: Event) => {
      try {
        const customEvent = event as CustomEvent;
        if (!customEvent.detail) {
          console.error('Received send message event without detail:', event);
          return;
        }

        const { content, metadata } = customEvent.detail;
        // Call via ref to always use the latest implementation
        sendStreamMessageRef.current(content, metadata);
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

    // Add event listener (only runs once on mount)
    window.addEventListener('chat:send-message', handleSendMessage);

    // Cleanup on unmount
    return () => {
      window.removeEventListener('chat:send-message', handleSendMessage);
      // FIX: Only abort if there's an active cleanup function
      if (cleanupRef.current) {
        cleanupRef.current();
        cleanupRef.current = null;
      }
    };
  }, [dispatch]); // Only depends on dispatch (stable from context)

  // FIX: Cleanup on unmount - use requestId guard to prevent aborting completed requests
  useEffect(() => {
    return () => {
      // Only abort if there's an active request (not already completed)
      if (cleanupRef.current && activeRequestIdRef.current) {
        cleanupRef.current();
        cleanupRef.current = null;
      }
    };
  }, []); // Empty deps - only run on mount/unmount

  return {
    isStreaming: state.isStreaming,
    error: state.error,
    sendStreamMessage,
  };
}