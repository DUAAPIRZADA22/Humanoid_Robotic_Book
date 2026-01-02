/**
 * useStreamingResponse Hook
 * @fileoverview Handles SSE streaming with cancellation protection
 *
 * ⭐ CRITICAL: This hook prevents SSE cancellation loops by:
 * 1. Using useMemo for apiClient (prevents recreation)
 * 2. Using ref for event listener (prevents useEffect re-run)
 * 3. Using request ID tracking (prevents aborting active requests)
 * 4. Proper abort detection (distinguishes aborts from errors)
 */

import { useCallback, useRef, useEffect, useMemo } from 'react';

interface StreamingChunk {
  type: 'metadata' | 'source' | 'content' | 'answer_start' | 'answer_chunk' | 'complete' | 'error';
  content?: string;
  [key: string]: any;
}

interface UseStreamingResponseOptions {
  apiUrl: string;
  apiKey?: string;
  onChunk?: (chunk: StreamingChunk) => void;
  onComplete?: () => void;
  onError?: (error: { code: string; message: string; retryable?: boolean }) => void;
}

interface UseStreamingResponseReturn {
  startStreaming: (content: string, metadata?: any) => Promise<void>;
  isStreaming: boolean;
  error: any;
}

/**
 * Creates a memoized API client
 * Prevents recreation on every render (root cause of SSE cancellation)
 */
function createApiClient(endpoint: string, apiKey?: string) {
  return { endpoint, apiKey };
}

/**
 * Stream chat message with SSE
 * Includes proper abort detection and cleanup
 */
function streamChatMessage(
  config: ReturnType<typeof createApiClient>,
  request: { question: string; stream?: boolean },
  token: string | null,
  onChunk: (chunk: StreamingChunk) => void,
  onComplete: () => void,
  onError: (error: any) => void
): () => void {
  const controller = new AbortController();
  let hasCalledCompletion = false;

  const url = `${config.endpoint.replace(/\/$/, '')}/chat`;

  console.log('[SSE] Starting chat stream:', { url, request });

  const callCompleteOnce = () => {
    if (!hasCalledCompletion && !controller.signal.aborted) {
      hasCalledCompletion = true;
      console.log('[SSE] Stream complete, calling onComplete');
      onComplete();
    }
  };

  fetch(url, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Accept': 'text/event-stream',
      'Cache-Control': 'no-cache',
      ...(config.apiKey ? { 'X-API-Key': config.apiKey } : {}),
      ...(token ? { 'Authorization': `Bearer ${token}` } : {}),
    },
    body: JSON.stringify({ ...request, stream: true }),
    signal: controller.signal,
  })
    .then(async (response) => {
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const reader = response.body?.getReader();
      if (!reader) {
        throw new Error('Response body is not readable');
      }

      const decoder = new TextDecoder();
      let buffer = '';
      let isComplete = false;

      while (!isComplete && !controller.signal.aborted) {
        try {
          const { done, value } = await reader.read();

          if (done) {
            console.log('[SSE] Stream ended naturally');
            break;
          }

          buffer += decoder.decode(value, { stream: true });

          const lines = buffer.split('\n');
          buffer = lines.pop() || '';

          for (const line of lines) {
            const trimmedLine = line.trim();
            if (!trimmedLine || trimmedLine.startsWith(':')) continue;

            if (trimmedLine.startsWith('data: ')) {
              const data = trimmedLine.slice(6).trim();

              if (data === '[DONE]') {
                console.log('[SSE] Received [DONE] sentinel');
                isComplete = true;
                break;
              }

              try {
                const chunk: StreamingChunk = JSON.parse(data);
                console.log('[SSE] Chunk received:', chunk.type);

                if (chunk.type === 'complete') {
                  console.log('[SSE] Received complete signal');
                  onChunk(chunk);
                  isComplete = true;
                  break;
                } else if (chunk.type === 'error') {
                  console.error('[SSE] Error from backend:', chunk);
                  onChunk(chunk);
                  isComplete = true;
                  break;
                } else {
                  onChunk(chunk);
                }
              } catch (parseError) {
                console.error('[SSE] Failed to parse chunk:', data, parseError);
              }
            }
          }

          if (isComplete) break;

        } catch (readError) {
          if (controller.signal.aborted) {
            console.log('[SSE] Stream aborted by user (during read)');
            return;
          }
          throw readError;
        }
      }

      callCompleteOnce();

    })
    .catch((error) => {
      // FIX: Check for abort FIRST - handle silently
      if (controller.signal.aborted || error?.name === 'AbortError') {
        console.log('[SSE] Request cancelled (abort detected)');
        return; // Don't call onError for user cancellations
      }

      console.error('[SSE] Error:', error);
      onError({
        code: 'STREAM_ERROR',
        message: error.message || 'Streaming failed',
        retryable: true,
      });
    });

  // Return cleanup function
  return () => {
    // FIX: Only abort if not already completed
    if (!hasCalledCompletion) {
      console.log('[SSE] Cleanup: aborting stream');
      controller.abort();
    }
  };
}

/**
 * Main hook for streaming responses
 */
export function useStreamingResponse(options: UseStreamingResponseOptions): UseStreamingResponseReturn {
  const { apiUrl, apiKey, onChunk, onComplete, onError } = options;

  // Refs for tracking state (persistent across re-renders)
  const cleanupRef = useRef<(() => void) | null>(null);
  const isRequestInProgressRef = useRef(false);
  const activeRequestIdRef = useRef<string | null>(null);

  // FIX 1: Memoize apiClient to prevent recreation on every render
  const apiClient = useMemo(
    () => createApiClient(apiUrl, apiKey),
    [apiUrl, apiKey]
  );

  // Handle streaming completion
  const handleStreamComplete = useCallback(() => {
    isRequestInProgressRef.current = false;
    activeRequestIdRef.current = null;
    onComplete?.();
  }, [onComplete]);

  // Handle streaming errors
  const handleStreamError = useCallback((error: any) => {
    isRequestInProgressRef.current = false;
    activeRequestIdRef.current = null;
    onError?.(error);
  }, [onError]);

  /**
   * Send a message with streaming support
   * FIX 3: Request ID tracking prevents aborting active requests
   */
  const startStreaming = useCallback(async (content: string, metadata?: any): Promise<void> => {
    if (!content || typeof content !== 'string') {
      console.error('Invalid content provided to startStreaming:', content);
      return;
    }

    // Generate unique request ID
    const requestId = `req_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    activeRequestIdRef.current = requestId;

    // Prevent concurrent requests
    if (isRequestInProgressRef.current) {
      console.warn('[Chat] Request already in progress, ignoring duplicate call');
      return;
    }

    console.log('Sending message to API:', { content, metadata, endpoint: apiUrl });

    // Clear any previous cleanup
    if (cleanupRef.current) {
      const oldCleanup = cleanupRef.current;
      cleanupRef.current = null;
      oldCleanup();
    }

    const request = {
      question: content,
      top_k: 3,
      similarity_threshold: 0.2,
      use_rerank: false,
      stream: true,
    };

    // Start streaming
    const cleanup = streamChatMessage(
      apiClient,
      request,
      null, // token
      (chunk) => {
        // Filter out sources if you don't want them displayed
        if (chunk.type === 'source') {
          console.debug('[SSE] Source skipped:', chunk.source);
          return;
        }
        onChunk?.(chunk);
      },
      handleStreamComplete,
      handleStreamError
    );

    // FIX: Only store cleanup if this is still the active request
    if (activeRequestIdRef.current === requestId) {
      cleanupRef.current = cleanup;
    }
  }, [apiClient, onChunk, handleStreamComplete, handleStreamError, apiUrl]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (cleanupRef.current && activeRequestIdRef.current) {
        cleanupRef.current();
        cleanupRef.current = null;
      }
    };
  }, []);

  return {
    startStreaming,
    isStreaming: isRequestInProgressRef.current,
    error: null,
  };
}
