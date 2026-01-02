---
name: chatbot-widget-creator
description: Creates a battle-tested ChatGPT-style chatbot widget with SSE streaming, RAG integration, and protection against common React issues like infinite re-renders and SSE cancellation.
category: frontend
version: 4.0.0
date_updated: 2025-12-28
---

# Chatbot Widget Creator Skill

## Purpose

Creates a **production-ready ChatGPT-style chatbot widget** with advanced features:
- **SSE Streaming** with cancellation protection (no more "Request cancelled" loops)
- **Immediate responses** - LLM starts streaming without waiting for RAG retrieval
- **Clean answers** - sources filtered out, no extra formatting
- **Infinite re-render protection** using useMemo and stable refs
- **Text selection "Ask AI"** functionality
- **RAG backend integration** ready
- **Performance monitoring** and debugging utilities

## Critical Lessons Learned (from real implementation)

### 1. **SSE Cancellation Prevention** ⭐ MOST IMPORTANT

**Problem**: SSE streams were being cancelled repeatedly with logs:
```
[SSE] Starting chat stream
[SSE] Cleanup: aborting stream
[SSE] Request cancelled
```

**Root Cause**: Dependency cascade:
1. `apiClient` recreated on every render (not memoized)
2. `sendStreamMessage` depended on `apiClient` → recreated
3. Event listener `useEffect` depended on `sendStreamMessage` → re-ran
4. Cleanup called on every re-run → aborted stream

**Solution** - Three critical fixes:

```typescript
// FIX 1: Memoize apiClient to prevent recreation
const apiClient = useMemo(
  () => createApiClient(chatApiEndpoint || '', chatApiKey || ''),
  [chatApiEndpoint, chatApiKey] // Only recreate when config actually changes
);

// FIX 2: Use ref-based event listener (never re-runs)
const sendStreamMessageRef = useRef(sendStreamMessage);
sendStreamMessageRef.current = sendStreamMessage;

useEffect(() => {
  const handler = (e) => sendStreamMessageRef.current(e.detail.content, e.detail.metadata);
  window.addEventListener('chat:send-message', handler);
  return () => {
    window.removeEventListener('chat:send-message', handler);
    if (cleanupRef.current) {
      cleanupRef.current();
      cleanupRef.current = null;
    }
  };
}, [dispatch]); // Only depends on stable dispatch

// FIX 3: Request ID tracking prevents aborting active requests
const activeRequestIdRef = useRef<string | null>(null);

const sendStreamMessage = useCallback(async (content: string) => {
  const requestId = `req_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  activeRequestIdRef.current = requestId;

  // ... after streaming starts
  if (activeRequestIdRef.current === requestId) {
    cleanupRef.current = cleanup;
  }
}, [apiClient, token, dispatch, ...]);
```

### 2. **Immediate Response Backend** ⭐ UX Critical

**Problem**: Long wait (8-16 seconds) before any response appears

**Solution**: Start LLM streaming immediately, do RAG in background:
```python
# Backend - stream immediately, retrieve in background
async def generate_response():
    yield metadata  # Send immediately

    # Start retrieval in background (non-blocking)
    retrieval_task = asyncio.create_task(retrieval_pipeline.retrieve(query))

    # Start LLM NOW without waiting for retrieval
    async for chunk in llm_service.generate_answer(query, context=[], stream=True):
        yield chunk  # Stream immediately

    # Wait for retrieval in background (fire and forget)
    await asyncio.wait_for(retrieval_task, timeout=5.0)
```

### 3. **Clean Answer Display** ⭐ User Experience

**Problem**: Responses showed sources and had extra formatting like `---` and `**AI Answer:**`

**Solution**: Filter at frontend chunk handler:
```typescript
const handleStreamChunk = useCallback((chunk: any) => {
  switch (chunk.type) {
    case 'source':
      // Skip sources entirely - don't display
      console.debug('[SSE] Source skipped:', chunk.source);
      break;

    case 'answer_start':
      // Don't add separator or label
      break;

    case 'content':
    case 'answer_chunk':
      // Append directly without extra formatting
      dispatch(chatActions.appendStream(chunk.content));
      break;

    case 'complete':
      // End streaming without trailing separator
      dispatch(chatActions.endStream());
      break;
  }
}, [dispatch]);
```

### 4. **Proper Abort Detection** ⭐ Error Handling

**Problem**: Aborts logged as errors, confusing debugging

**Solution**: Distinguish aborts from errors:
```typescript
// In stream handler
.catch((error) => {
  // Check for abort FIRST - handle silently
  if (controller.signal.aborted || error?.name === 'AbortError') {
    console.log('[SSE] Request cancelled (abort detected)');
    return; // Don't call onError for user cancellations
  }

  // Only then handle actual errors
  onError(chatError);
});

// Cleanup function
return () => {
  if (!hasCalledCompletion) {
    console.log('[SSE] Cleanup: aborting stream');
    controller.abort();
  }
};
```

## Prerequisites

1. **Backend Requirements**:
   - FastAPI or similar with SSE support
   - Endpoint: `POST /chat` returning Server-Sent Events
   - Request format: `{ "question": string, "stream": true }`
   - Response format: SSE with chunks

2. **Frontend Dependencies**:
   ```bash
   npm install react framer-motion react-markdown remark-gfm
   ```

## Quick Start

### 1. Create the Widget Structure

```bash
mkdir -p src/components/ChatWidget/{components,hooks,contexts,utils,styles}
cp -r .claude/skills/chatbot-widget-creator/templates/* src/components/ChatWidget/
```

### 2. Backend API Requirements

Your backend must implement **immediate streaming**:

```python
@app.post("/chat")
async def chat_stream(request: ChatRequest):
    async def generate_response():
        # 1. Send metadata IMMEDIATELY
        yield f"data: {json.dumps({'type': 'metadata'})}\n\n"

        # 2. Start retrieval in background (don't await)
        retrieval_task = asyncio.create_task(retrieval_pipeline.retrieve(query))

        # 3. Start LLM NOW - don't wait for retrieval
        yield f"data: {json.dumps({'type': 'answer_start'})}\n\n"

        async for chunk in llm_service.generate_answer(query, context=[], stream=True):
            yield f"data: {json.dumps({'type': 'answer_chunk', 'content': chunk})}\n\n"

        # 4. Complete
        yield f"data: {json.dumps({'type': 'complete'})}\n\n"

        # 5. Wait for retrieval in background (optional)
        await asyncio.wait_for(retrieval_task, timeout=5.0)

    return StreamingResponse(generate_response(), media_type="text/event-stream")
```

### 3. Integration

Add to your site root (e.g., `src/theme/Root.tsx`):

```tsx
import React from 'react';
import ChatWidget from '../components/ChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

## Architecture Details

### Core Components

1. **ChatWidget**: Main entry point with ChatProvider
2. **ChatInterface**: Chat UI container
3. **MessageBubble**: Message display (React.memo optimized)
4. **MessageInput**: User input with submit handler
5. **ThinkingIndicator**: Loading state indicator

### Hooks

1. **useChatStream**: ⭐ **CRITICAL** - Handles SSE with cancellation protection
2. **useTextSelection**: Detect and handle text selection
3. **useChatContext**: Access chat state and actions

### Key Patterns

#### 1. Stable Event Listener Pattern
```typescript
// Store callback in ref (updates without re-render)
const callbackRef = useRef(callback);
callbackRef.current = callback;

// useEffect only runs once on mount
useEffect(() => {
  const handler = (e) => callbackRef.current(e);
  window.addEventListener('event', handler);
  return () => window.removeEventListener('event', handler);
}, []); // Empty deps = runs once
```

#### 2. Request ID Tracking
```typescript
const activeRequestIdRef = useRef<string | null>(null);

const sendRequest = async () => {
  const requestId = `req_${Date.now()}`;
  activeRequestIdRef.current = requestId;

  // After async operation starts
  if (activeRequestIdRef.current === requestId) {
    // Only set cleanup if this is still the active request
    cleanupRef.current = cleanup;
  }
};
```

#### 3. Memoized API Client
```typescript
const apiClient = useMemo(
  () => createApiClient(endpoint, apiKey),
  [endpoint, apiKey] // Only recreate when these change
);
```

## Troubleshooting

### SSE Cancellation Loop
**Symptoms**: `[SSE] Starting chat stream`, `[SSE] Cleanup: aborting stream` repeating

**Solutions**:
1. ✅ Memoize `apiClient` with useMemo
2. ✅ Use ref for event listener callback
3. ✅ Add request ID tracking
4. ✅ Only depend on stable values in useEffect

### Slow First Response
**Symptoms**: 8-16 second delay before any text appears

**Solution**: Backend should start LLM immediately, do retrieval in background

### Sources Showing in Answers
**Symptoms**: Answers include `**Source:** ...` sections

**Solution**: Skip `source` type chunks in frontend handler

### Aborts Logged as Errors
**Symptoms**: `[SSE] Request cancelled` appears as error

**Solution**: Check `controller.signal.aborted` first, return silently if true

## Production Checklist

- [ ] Memoize all API clients and expensive computations
- [ ] Use ref pattern for event listener callbacks
- [ ] Implement request ID tracking for concurrent requests
- [ ] Start LLM streaming immediately (don't wait for RAG)
- [ ] Filter source chunks from display
- [ ] Proper abort detection (check signal before error handling)
- [ ] Test with React 18+ StrictMode (double useEffect)
- [ ] Monitor for memory leaks (unclosed AbortControllers)
- [ ] Test on slow networks
- [ ] Verify CORS includes `text/event-stream`

## Result

A production-ready chat widget that:
- Never aborts streams due to re-renders
- Shows immediate response to user input
- Displays clean answers without source clutter
- Handles all edge cases gracefully
- Maintains performance over time
