/**
 * API type definitions for chat widget
 * @fileoverview TypeScript definitions for API communication
 */

export interface ApiConfig {
  /** API base URL */
  endpoint: string;
  /** Authentication key */
  apiKey: string;
  /** Request timeout (ms) */
  timeout: number;
  /** Retry attempts */
  retries: number;
  /** Additional headers */
  headers: Record<string, string>;
}

export interface ChatRequest {
  /** User's question or message */
  question: string;
  /** Optional context */
  context?: RequestContext;
  /** Whether to stream the response */
  stream: boolean;
  /** Optional session ID for conversation continuity */
  sessionId?: string;
}

export interface RequestContext {
  /** Text selected by the user (max 2000 chars) */
  selectedText?: string;
  /** Current page URL */
  pageUrl?: string;
  /** Current page title */
  pageTitle?: string;
}

export interface ChatResponse {
  /** Session identifier */
  sessionId: string;
  /** AI response message */
  response: ChatMessage;
  /** Token usage information */
  usage?: TokenUsage;
}

export interface ChatMessage {
  /** Unique message identifier */
  id: string;
  /** Message content (may contain markdown) */
  content: string;
  /** Role of the message sender */
  role: 'user' | 'assistant' | 'system';
  /** Message timestamp */
  timestamp: string;
  /** Optional metadata */
  metadata?: MessageMetadata;
}

export interface MessageMetadata {
  /** Original text selection that prompted this message */
  selectedText?: string;
  /** Page context for the message */
  context?: string;
  /** Number of tokens in the message */
  tokens?: number;
}

export interface StreamingChunk {
  /** Type of streaming chunk */
  type: 'start' | 'chunk' | 'error' | 'done';
  /** Content chunk (only for type: 'chunk') */
  content?: string;
  /** Error message (only for type: 'error') */
  error?: string;
  /** Session identifier */
  sessionId?: string;
  /** Chunk timestamp */
  timestamp: string;
}

export interface TokenUsage {
  /** Number of tokens in the prompt */
  promptTokens: number;
  /** Number of tokens in the completion */
  completionTokens: number;
  /** Total number of tokens */
  totalTokens: number;
}

export interface HealthResponse {
  /** Health status */
  status: 'healthy' | 'degraded' | 'unhealthy';
  /** API version */
  version?: string;
  /** Health check timestamp */
  timestamp: string;
}

export interface ErrorResponse {
  /** Error details */
  error: {
    /** Error code */
    code: string;
    /** Human-readable error message */
    message: string;
    /** Additional error details */
    details?: Record<string, any>;
  };
  /** Error timestamp */
  timestamp: string;
}

/** API error types */
export enum ApiErrorType {
  NETWORK_ERROR = 'NETWORK_ERROR',
  TIMEOUT_ERROR = 'TIMEOUT_ERROR',
  AUTHENTICATION_ERROR = 'AUTHENTICATION_ERROR',
  RATE_LIMIT_ERROR = 'RATE_LIMIT_ERROR',
  SERVER_ERROR = 'SERVER_ERROR',
  INVALID_REQUEST = 'INVALID_REQUEST',
}

/** API response wrapper for error handling */
export type ApiResponse<T = any> = {
  data?: T;
  error?: ErrorResponse;
  status: number;
  ok: boolean;
};