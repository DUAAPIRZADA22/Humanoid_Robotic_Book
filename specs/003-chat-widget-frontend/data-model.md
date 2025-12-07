# Data Model: Chat Widget Frontend

**Date**: 2025-12-07
**Feature**: 003-chat-widget-frontend

## Core Entities

### 1. ChatMessage

Represents a single message in the chat conversation.

```typescript
interface ChatMessage {
  id: string;                    // Unique identifier (timestamp-based)
  content: string;               // Message content (markdown supported)
  role: 'user' | 'assistant' | 'system';  // Sender role
  timestamp: Date;               // Creation time
  isStreaming?: boolean;         // True if currently streaming
  metadata?: MessageMetadata;    // Optional metadata
}

interface MessageMetadata {
  selectedText?: string;         // Original text selection if applicable
  context?: string;             // Page context for the message
  tokens?: number;              // Token count (for analytics)
}
```

**Validation Rules**:
- `id`: Required, unique, string
- `content`: Required, non-empty, max 10,000 characters
- `role`: Required, enum value
- `timestamp`: Required, valid Date
- `isStreaming`: Optional, boolean
- `metadata`: Optional, nested object

### 2. ChatSession

Represents the overall chat session state.

```typescript
interface ChatSession {
  id: string;                    // Session identifier
  messages: ChatMessage[];       // Ordered message list
  isOpen: boolean;               // Widget visibility state
  isTyping: boolean;             // Assistant typing indicator
  error: ChatError | null;       // Current error state
  settings: SessionSettings;     // Session configuration
}

interface ChatError {
  code: string;                  // Error code
  message: string;               // User-friendly message
  retryable: boolean;            // Can user retry?
  timestamp: Date;               // When error occurred
}

interface SessionSettings {
  maxMessages: number;           // Max messages to keep (default: 100)
  autoSave: boolean;             // Auto-save to localStorage
  streamingEnabled: boolean;     // Enable SSE streaming
}
```

**State Transitions**:
```
Initial → Loading → Ready/Streaming
         ↘ Error → Ready/Streaming
```

### 3. Selection

Represents a text selection on the page.

```typescript
interface TextSelection {
  id: string;                    // Unique selection ID
  text: string;                  // Selected text content
  range: SelectionRange;         // DOM range information
  position: SelectionPosition;   // Calculated position
  timestamp: Date;               // Selection time
}

interface SelectionRange {
  startOffset: number;           // Start character offset
  endOffset: number;             // End character offset
  collapsed: boolean;            // Whether selection is collapsed
}

interface SelectionPosition {
  top: number;                   // Y position relative to viewport
  left: number;                  // X position relative to viewport
  width: number;                 // Selection width
  height: number;                // Selection height
}
```

**Constraints**:
- `text`: Max 2000 characters (truncated with warning)
- Position calculated using `getBoundingClientRect()`
- Debounced selection events (300ms delay)

### 4. ApiConfig

Configuration for API communication.

```typescript
interface ApiConfig {
  endpoint: string;              // API base URL
  apiKey: string;                // Authentication key
  timeout: number;               // Request timeout (ms)
  retries: number;               // Retry attempts
  headers: Record<string, string>; // Additional headers
}

interface ChatRequest {
  question: string;              // User question
  context?: RequestContext;      // Optional context
  stream: boolean;               // Enable streaming
}

interface RequestContext {
  selectedText?: string;         // Selected text context
  pageUrl?: string;              // Current page URL
  pageTitle?: string;            // Current page title
}
```

### 5. StreamingResponse

Represents a streaming response chunk.

```typescript
interface StreamingResponse {
  type: 'start' | 'chunk' | 'error' | 'done';  // Response type
  content?: string;            // Content chunk (for 'chunk' type)
  error?: string;              // Error message (for 'error' type)
  sessionId?: string;          // Session identifier
  timestamp: Date;             // Response timestamp
}
```

## State Management Schema

### ChatState (useReducer)

```typescript
interface ChatState {
  messages: ChatMessage[];       // All messages in order
  isLoading: boolean;            // Loading state
  isStreaming: boolean;          // Currently streaming
  streamingId?: string;          // Current streaming message ID
  error: ChatError | null;       // Current error
  settings: SessionSettings;     // Session settings
}

type ChatAction =
  | { type: 'ADD_MESSAGE'; payload: ChatMessage }
  | { type: 'START_STREAM'; payload: string }
  | { type: 'APPEND_STREAM'; payload: string }
  | { type: 'END_STREAM' }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: ChatError | null }
  | { type: 'CLEAR_MESSAGES' }
  | { type: 'UPDATE_SETTINGS'; payload: Partial<SessionSettings> };
```

### LocalStorage Schema

```typescript
interface StorageEntry {
  version: string;               // Storage version
  timestamp: number;             // Last updated
  data: {
    messages: ChatMessage[];     // Persisted messages
    settings: SessionSettings;  // User settings
  };
}
```

## Relationships

```
ChatSession
  ├── contains[0..*] ChatMessage
  ├── has[1] ApiConfig
  └── references[0..1] TextSelection

ChatMessage
  ├── references[0..1] TextSelection (via metadata)
  └── belongs_to[1] ChatSession

TextSelection
  ├── triggers[1] ChatRequest
  └── converts_to[1] ChatMessage
```

## Data Flow

1. **User Input** → ChatRequest → API
2. **API Response** → StreamingResponse → ChatMessage
3. **Text Selection** → TextSelection → ChatRequest
4. **ChatMessage** → localStorage (encrypted)
5. **localStorage** → ChatSession (on load)

## Validation Schemas

### ChatMessage Validation
```typescript
const messageSchema = {
  id: { type: 'string', required: true },
  content: { type: 'string', minLength: 1, maxLength: 10000, required: true },
  role: { type: 'string', enum: ['user', 'assistant', 'system'], required: true },
  timestamp: { type: 'date', required: true },
  isStreaming: { type: 'boolean', optional: true }
};
```

### ApiConfig Validation
```typescript
const configSchema = {
  endpoint: { type: 'url', required: true },
  apiKey: { type: 'string', minLength: 1, required: true },
  timeout: { type: 'number', min: 1000, max: 30000, default: 10000 },
  retries: { type: 'number', min: 0, max: 5, default: 3 }
};
```

## Performance Considerations

### Message List Optimization
- Use `React.memo` for MessageBubble component
- Virtual scrolling for >50 messages
- Lazy loading of message history

### State Updates
- Batch multiple updates with `unstable_batchedUpdates`
- Use refs for values that change frequently (streaming content)
- Debounce expensive calculations

### Storage Optimization
- Limit stored messages (keep last 100)
- Compress old messages after 30 days
- Use IndexedDB for large conversations if needed

## Security Model

### Data Protection
- Encrypt localStorage data (AES-256)
- Never log sensitive content
- Clear sensitive data on logout

### Input Sanitization
- Sanitize HTML in markdown rendering
- Escape special characters
- Limit message length

### API Security
- Use HTTPS for all requests
- Include API key in headers (not URL)
- Implement request signing if needed