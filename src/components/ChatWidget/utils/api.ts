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
  timeout: 10000,
  retries: 3,
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
 * Send chat message with streaming support
 * @param config - API configuration
 * @param request - Chat request payload
 * @param onChunk - Callback for streaming chunks
 * @param onComplete - Callback for completion
 * @param onError - Callback for errors
 * @returns Cleanup function
 */
export function streamChatMessage(
  config: ApiConfig,
  request: ChatRequest,
  onChunk: (chunk: StreamingChunk) => void,
  onComplete: () => void,
  onError: (error: ChatError) => void
): () => void {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), config.timeout);

  const url = `${config.endpoint.replace(/\/$/, '')}/chat`;

  console.log('API Request Details:', {
    url,
    method: 'POST',
    config,
    request
  });

  fetch(url, {
    method: 'POST',
    headers: {
      ...config.headers,
      'X-API-Key': config.apiKey,
      'Content-Type': 'application/json',
      'Accept': 'text/event-stream',
      'Cache-Control': 'no-cache',
    },
    body: JSON.stringify(request),
    signal: controller.signal,
  })
    .then(response => {
      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const reader = response.body?.getReader();
      const decoder = new TextDecoder();

      if (!reader) {
        throw new Error('Response body is not readable');
      }

      let buffer = '';

      function processChunk(): Promise<void> {
        return reader.read().then(({ done, value }) => {
          if (done) {
            onComplete();
            return;
          }

          buffer += decoder.decode(value, { stream: true });
          const lines = buffer.split('\n');
          buffer = lines.pop() || '';

          for (const line of lines) {
            if (line.trim() === '') continue;

            if (line.startsWith('data: ')) {
              try {
                const data = line.slice(6);
                if (data === '[DONE]') {
                  onComplete();
                  return;
                }

                const chunk: StreamingChunk = JSON.parse(data);
                onChunk(chunk);
              } catch (error) {
                console.error('Failed to parse chunk:', error);
              }
            }
          }

          return processChunk();
        });
      }

      return processChunk();
    })
    .catch(error => {
      clearTimeout(timeoutId);

      let chatError: ChatError;

      if (error instanceof Error) {
        if (error.name === 'AbortError') {
          chatError = createApiError(
            ApiErrorType.TIMEOUT_ERROR,
            'Request timed out',
            false
          );
        } else if (error.message.includes('401')) {
          chatError = createApiError(
            ApiErrorType.AUTHENTICATION_ERROR,
            'Invalid API key',
            false
          );
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

      onError(chatError);
    });

  // Return cleanup function
  return () => {
    clearTimeout(timeoutId);
    controller.abort();
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