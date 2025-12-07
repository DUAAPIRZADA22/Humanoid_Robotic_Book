/**
 * Hook for handling chat streaming via Server-Sent Events
 * @fileoverview Custom hook for managing SSE connections and message streaming
 */

import { useCallback, useRef, useEffect } from 'react';
import { useChatContext } from '../contexts/ChatContext';
import { createApiClient, streamChatMessage } from '../utils/api';
import { useDocusaurusContext } from '@docusaurus/useDocusaurusContext';
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

  // Create API client from Docusaurus config
  const apiClient = createApiClient(
    siteConfig.customFields.chatApiEndpoint as string,
    siteConfig.customFields.chatApiKey as string
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
      context: metadata?.selectedText ? {
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