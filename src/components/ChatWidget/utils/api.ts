/**
 * API client for chat widget communication
 * @fileoverview HTTP client with authentication and error handling
 */

import {
  ApiConfig,
  ChatRequest,
  StreamingChunk,
  ApiResponse,
  ErrorResponse,
  ApiErrorType,
  HealthResponse,
} from '../types/api';
import { ChatError } from '../types/chat';

/**
 * Default API configuration
 */
const DEFAULT_CONFIG: Partial<ApiConfig> = {
  timeout: 300000,  // Increased to 5 minutes for OpenRouter free tier API calls
  retries: 2,     // Reduced retries to avoid delay
  headers: {
    'Content-Type': 'application/json',
  },
};

/**
 * Create API error object
 * @param type - Error type
 * @param message - Error message
 * @param retryable - Whether the error is retryable
 * @returns ChatError object
 */
function createApiError(
  type: ApiErrorType,
  message: string,
  retryable: boolean = false
): ChatError {
  return {
    code: type,
    message,
    retryable,
    timestamp: new Date(),
  };
}

/**
 * Exponential backoff delay calculation
 * @param attempt - Current attempt number
 * @returns Delay in milliseconds
 */
function getBackoffDelay(attempt: number): number {
  return Math.min(1000 * Math.pow(2, attempt), 30000);
}

/**
 * Make HTTP request with retries and error handling
 * @param config - API configuration
 * @param endpoint - API endpoint path
 * @param options - Request options
 * @param attempt - Current retry attempt
 * @returns Promise resolving to response
 */
async function makeRequest<T>(
  config: ApiConfig,
  endpoint: string,
  options: RequestInit,
  attempt: number = 0
): Promise<ApiResponse<T>> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), config.timeout);

  try {
    const url = `${config.endpoint.replace(/\/$/, '')}/${endpoint.replace(/^\//, '')}`;

    const response = await fetch(url, {
      ...options,
      signal: controller.signal,
      headers: {
        ...config.headers,
        'X-API-Key': config.apiKey,
        ...options.headers,
      },
    });

    clearTimeout(timeoutId);

    // Handle non-OK responses
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({})) as ErrorResponse;

      // Map HTTP status to error types
      let errorType: ApiErrorType;
      let retryable = false;

      switch (response.status) {
        case 401:
          errorType = ApiErrorType.AUTHENTICATION_ERROR;
          retryable = false;
          break;
        case 429:
          errorType = ApiErrorType.RATE_LIMIT_ERROR;
          retryable = true;
          break;
        case 500:
        case 502:
        case 503:
        case 504:
          errorType = ApiErrorType.SERVER_ERROR;
          retryable = true;
          break;
        case 400:
        case 422:
          errorType = ApiErrorType.INVALID_REQUEST;
          retryable = false;
          break;
        default:
          errorType = ApiErrorType.NETWORK_ERROR;
          retryable = true;
      }

      const message = errorData.error?.message || `HTTP ${response.status}: ${response.statusText}`;

      // Retry if possible and we haven't exceeded max retries
      if (retryable && attempt < config.retries) {
        const delay = getBackoffDelay(attempt);
        await new Promise(resolve => setTimeout(resolve, delay));
        return makeRequest<T>(config, endpoint, options, attempt + 1);
      }

      return {
        error: errorData.error || {
          error: {
            code: errorType,
            message,
            timestamp: new Date().toISOString(),
          },
        },
        status: response.status,
        ok: false,
      };
    }

    // Parse successful response
    const data = await response.json();

    return {
      data,
      status: response.status,
      ok: true,
    };
  } catch (error) {
    clearTimeout(timeoutId);

    // Handle network errors and timeouts
    let errorType: ApiErrorType;
    let message: string;

    if (error instanceof Error) {
      if (error.name === 'AbortError') {
        errorType = ApiErrorType.TIMEOUT_ERROR;
        message = 'Request timed out';
      } else {
        errorType = ApiErrorType.NETWORK_ERROR;
        message = 'Network error occurred';
      }
    } else {
      errorType = ApiErrorType.NETWORK_ERROR;
      message = 'Unknown error occurred';
    }

    // Retry if possible
    if (attempt < config.retries) {
      const delay = getBackoffDelay(attempt);
      await new Promise(resolve => setTimeout(resolve, delay));
      return makeRequest<T>(config, endpoint, options, attempt + 1);
    }

    return {
      error: {
        error: {
          code: errorType,
          message,
          timestamp: new Date().toISOString(),
        },
      },
      status: 0,
      ok: false,
    };
  }
}

/**
 * Send chat message with streaming support using SSE (Server-Sent Events)
 *
 * This function:
 * - Uses fetch() + ReadableStream for proper SSE handling
 * - Parses SSE format: "data: {...}\n\n"
 * - Detects backend "complete" event to stop reading
 * - Handles incremental UI updates via onChunk callback
 * - FIX: Properly detects aborts vs errors to prevent false error logging
 *
 * @param config - API configuration
 * @param request - Chat request payload
 * @param token - Auth token (optional, can be null)
 * @param onChunk - Callback for each streaming chunk
 * @param onComplete - Callback when stream completes
 * @param onError - Callback for errors
 * @returns Cleanup function to abort the stream
 */
export function streamChatMessage(
  config: ApiConfig,
  request: ChatRequest,
  token: string | null,
  onChunk: (chunk: StreamingChunk) => void,
  onComplete: () => void,
  onError: (error: ChatError) => void
): () => void {
  const controller = new AbortController();
  let hasCalledCompletion = false; // FIX: Track if onComplete was called

  // NOTE: No timeout for streaming - let the backend control completion
  // The backend sends "type":"complete" when done

  const url = `${config.endpoint.replace(/\/$/, '')}/chat`;

  console.log('[SSE] Starting chat stream:', { url, request });

  // FIX: Helper to call completion exactly once
  const callCompleteOnce = () => {
    if (!hasCalledCompletion && !controller.signal.aborted) {
      hasCalledCompletion = true;
      console.log('[SSE] Stream complete, calling onComplete');
      onComplete();
    }
  };

  // Start the fetch request
  fetch(url, {
    method: 'POST',
    headers: {
      ...config.headers,
      'X-API-Key': config.apiKey,
      ...(token ? { 'Authorization': `Bearer ${token}` } : {}),
      'Content-Type': 'application/json',
      'Accept': 'text/event-stream',
      'Cache-Control': 'no-cache',
    },
    body: JSON.stringify(request),
    signal: controller.signal,
  })
    .then(async (response) => {
      // Check for HTTP errors
      if (!response.ok) {
        if (response.status === 401) {
          window.dispatchEvent(new CustomEvent('auth:required'));
        }
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      // Get the ReadableStream reader from response body
      const reader = response.body?.getReader();
      if (!reader) {
        throw new Error('Response body is not readable');
      }

      const decoder = new TextDecoder();
      let buffer = '';
      let isComplete = false;

      // Read chunks from the stream
      while (!isComplete && !controller.signal.aborted) {
        try {
          const { done, value } = await reader.read();

          // Stream ended naturally
          if (done) {
            console.log('[SSE] Stream ended naturally');
            break;
          }

          // Decode the chunk and add to buffer
          buffer += decoder.decode(value, { stream: true });

          // Split by double newline (SSE message boundary)
          const lines = buffer.split('\n');
          // Keep the last partial line in buffer (might be incomplete)
          buffer = lines.pop() || '';

          // Process each complete line
          for (const line of lines) {
            // Skip empty lines and comments
            const trimmedLine = line.trim();
            if (!trimmedLine || trimmedLine.startsWith(':')) {
              continue;
            }

            // Parse SSE data line: "data: {...}"
            if (trimmedLine.startsWith('data: ')) {
              const data = trimmedLine.slice(6).trim();

              // Check for [DONE] sentinel (OpenAI-style, not used by your backend)
              if (data === '[DONE]') {
                console.log('[SSE] Received [DONE] sentinel');
                isComplete = true;
                break;
              }

              try {
                // Parse JSON chunk
                const chunk: StreamingChunk = JSON.parse(data);
                console.log('[SSE] Chunk received:', chunk.type);

                // Handle chunk types
                if (chunk.type === 'complete') {
                  // Backend signals completion - stop reading
                  console.log('[SSE] Received complete signal');
                  onChunk(chunk); // Notify UI of completion
                  isComplete = true;
                  break;
                } else if (chunk.type === 'error') {
                  // Backend sent an error
                  console.error('[SSE] Error from backend:', chunk);
                  onChunk(chunk);
                  isComplete = true;
                  break;
                } else {
                  // Normal chunk (metadata, source, answer_chunk, etc.)
                  onChunk(chunk);
                }
              } catch (parseError) {
                console.error('[SSE] Failed to parse chunk:', data, parseError);
                // Continue processing other chunks
              }
            }
          }

          // Exit loop if complete signal received
          if (isComplete) {
            break;
          }

        } catch (readError) {
          // FIX: Check if this is an abort error BEFORE re-throwing
          // This prevents the error handler from treating aborts as errors
          if (controller.signal.aborted) {
            console.log('[SSE] Stream aborted by user (during read)');
            return;
          }
          throw readError;
        }
      }

      // Signal completion to caller (only if not aborted)
      callCompleteOnce();

    })
    .catch((error) => {
      // FIX: First check if this was an abort - handle silently
      if (controller.signal.aborted || error?.name === 'AbortError') {
        console.log('[SSE] Request cancelled (abort detected)');
        return; // Don't call onError for user cancellations
      }

      // Handle fetch/read errors
      let chatError: ChatError;

      if (error instanceof Error) {
        if (error.message.includes('401')) {
          chatError = createApiError(
            ApiErrorType.AUTHENTICATION_ERROR,
            'Authentication required. Please sign in.',
            false
          );
          window.dispatchEvent(new CustomEvent('auth:required'));
        } else if (error.message.includes('429')) {
          chatError = createApiError(
            ApiErrorType.RATE_LIMIT_ERROR,
            'Too many requests. Please try again later.',
            true
          );
        } else {
          chatError = createApiError(
            ApiErrorType.NETWORK_ERROR,
            error.message,
            true
          );
        }
      } else {
        chatError = createApiError(
          ApiErrorType.NETWORK_ERROR,
          'Unknown error occurred',
          true
        );
      }

      console.error('[SSE] Error:', chatError);
      onError(chatError);
    });

  // Return cleanup function
  return () => {
    // FIX: Only log and abort if we haven't already completed
    if (!hasCalledCompletion) {
      console.log('[SSE] Cleanup: aborting stream');
      controller.abort();
    }
  };
}

/**
 * Check API health
 * @param config - API configuration
 * @returns Promise resolving to health status
 */
export async function checkHealth(
  config: ApiConfig
): Promise<ApiResponse<HealthResponse>> {
  return makeRequest<HealthResponse>(config, 'health', {
    method: 'GET',
  });
}

/**
 * Create API client with configuration
 * @param endpoint - API base URL
 * @param apiKey - API authentication key
 * @param options - Additional configuration options
 * @returns Configured API client
 */
export function createApiClient(
  endpoint?: string | null,
  apiKey?: string | null,
  options: Partial<ApiConfig> = {}
): ApiConfig {
  return {
    ...DEFAULT_CONFIG,
    ...options,
    endpoint: endpoint || null,
    apiKey: apiKey || null,
  };
}