/**
 * Core chat widget types based on data model
 * @fileoverview TypeScript definitions for chat widget entities
 */

export interface ChatMessage {
  /** Unique identifier (timestamp-based) */
  id: string;
  /** Message content (markdown supported) */
  content: string;
  /** Sender role */
  role: 'user' | 'assistant' | 'system';
  /** Creation time */
  timestamp: Date;
  /** True if currently streaming */
  isStreaming?: boolean;
  /** Optional metadata */
  metadata?: MessageMetadata;
}

export interface MessageMetadata {
  /** Original text selection if applicable */
  selectedText?: string;
  /** Page context for the message */
  context?: string;
  /** Token count (for analytics) */
  tokens?: number;
}

export interface ChatSession {
  /** Session identifier */
  id: string;
  /** Ordered message list */
  messages: ChatMessage[];
  /** Widget visibility state */
  isOpen: boolean;
  /** Assistant typing indicator */
  isTyping: boolean;
  /** Current error state */
  error: ChatError | null;
  /** Session configuration */
  settings: SessionSettings;
}

export interface ChatError {
  /** Error code */
  code: string;
  /** User-friendly message */
  message: string;
  /** Can user retry? */
  retryable: boolean;
  /** When error occurred */
  timestamp: Date;
}

export interface SessionSettings {
  /** Max messages to keep (default: 100) */
  maxMessages: number;
  /** Auto-save to localStorage */
  autoSave: boolean;
  /** Enable SSE streaming */
  streamingEnabled: boolean;
}

export interface TextSelection {
  /** Unique selection ID */
  id: string;
  /** Selected text content */
  text: string;
  /** DOM range information */
  range: SelectionRange;
  /** Calculated position */
  position: SelectionPosition;
  /** Selection time */
  timestamp: Date;
}

export interface SelectionRange {
  /** Start character offset */
  startOffset: number;
  /** End character offset */
  endOffset: number;
  /** Whether selection is collapsed */
  collapsed: boolean;
}

export interface SelectionPosition {
  /** Y position relative to viewport */
  top: number;
  /** X position relative to viewport */
  left: number;
  /** Selection width */
  width: number;
  /** Selection height */
  height: number;
}

export interface StreamingResponse {
  /** Response type */
  type: 'start' | 'chunk' | 'error' | 'done';
  /** Content chunk (for 'chunk' type) */
  content?: string;
  /** Error message (for 'error' type) */
  error?: string;
  /** Session identifier */
  sessionId?: string;
  /** Response timestamp */
  timestamp: Date;
}

export interface ChatState {
  /** All messages in order */
  messages: ChatMessage[];
  /** Loading state */
  isLoading: boolean;
  /** Currently streaming */
  isStreaming: boolean;
  /** Current streaming message ID */
  streamingId?: string;
  /** Current error */
  error: ChatError | null;
  /** Session settings */
  settings: SessionSettings;
}

export type ChatAction =
  | { type: 'ADD_MESSAGE'; payload: ChatMessage }
  | { type: 'START_STREAM'; payload: string }
  | { type: 'APPEND_STREAM'; payload: string }
  | { type: 'END_STREAM' }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: ChatError | null }
  | { type: 'CLEAR_MESSAGES' }
  | { type: 'UPDATE_SETTINGS'; payload: Partial<SessionSettings> };

export interface StorageEntry {
  /** Storage version */
  version: string;
  /** Last updated */
  timestamp: number;
  /** Stored data */
  data: {
    /** Persisted messages */
    messages: ChatMessage[];
    /** User settings */
    settings: SessionSettings;
  };
}