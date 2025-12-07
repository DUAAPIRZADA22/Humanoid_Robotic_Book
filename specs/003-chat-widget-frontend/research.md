# Research Findings: Chat Widget Frontend Implementation

**Date**: 2025-12-07
**Feature**: 003-chat-widget-frontend
**Research Focus**: React patterns for streaming chat with glassmorphic design

## Key Decisions

### 1. Server-Sent Events (SSE) Implementation
**Decision**: Custom SSE implementation with useReducer for state management
**Rationale**: Prevents infinite re-renders, provides better performance than multiple useState hooks
**Alternatives considered**:
- WebSocket libraries (overkill for one-way streaming)
- Polling (inefficient for real-time updates)

### 2. State Management Pattern
**Decision**: useReducer with consolidated state object
**Rationale**: Single source of truth, atomic updates, prevents context fragmentation
**Alternatives considered**:
- Multiple useState hooks (causes re-render issues)
- Zustand/Redux (overkill for component-level state)

### 3. Glassmorphic Design
**Decision**: Custom CSS with backdrop-filter and Tailwind utilities
**Rationale**: Full control over appearance, lightweight, no additional runtime cost
**Alternatives considered**:
- CSS-in-JS libraries (unnecessary abstraction)
- Pre-built glassmorphism kits (limited customization)

### 4. Text Selection Detection
**Decision**: Native Selection API with debounced event handlers
**Rationale**: Browser-native performance, precise positioning control
**Alternatives considered**:
- Text selection libraries (added complexity)

### 5. Local Storage Persistence
**Decision**: Encrypted localStorage with crypto-js
**Rationale**: Client-side persistence without server dependency, security for chat history
**Alternatives considered**:
- IndexedDB (more complex than needed)
- Session storage (doesn't persist across sessions)

### 6. Authentication Strategy
**Decision**: API key in X-API-Key header with secure storage
**Rationale**: Simple, stateless, aligns with backend requirements
**Alternatives considered**:
- JWT tokens (requires refresh logic)
- Cookie-based (server-side setup complexity)

### 7. Performance Optimizations
**Decision**: React.memo for MessageBubble, virtual scrolling for long conversations
**Rationale**: Prevents unnecessary re-renders, handles 100+ messages efficiently
**Alternatives considered**:
- Manual pagination (breaks conversation flow)

## Implementation Patterns

### SSE Hook Pattern
```typescript
const useChatStream = (endpoint: string, apiKey: string) => {
  const [state, dispatch] = useReducer(chatReducer, initialState);

  // Consolidated state prevents fragmentation
  // Stable callbacks with useRef for streaming ID
  // AbortController for cleanup
};
```

### Glassmorphism CSS Pattern
```css
.glass-card {
  background: rgba(255, 255, 255, 0.05);
  backdrop-filter: blur(20px) saturate(180%);
  border: 1px solid rgba(255, 255, 255, 0.125);
}
```

### Text Selection Pattern
```typescript
const calculatePosition = useCallback(() => {
  // Edge detection for viewport boundaries
  // Scroll position consideration
  // Debounced selection handling
}, []);
```

## Anti-Patterns to Avoid

1. **Multiple useState for chat state** → Use useReducer
2. **Creating EventSource on every render** → Use refs and cleanup
3. **Inline functions in render** → Use useCallback
4. **Storing API keys in plain localStorage** → Use encryption or env vars
5. **Ignoring CORS preflight** → Configure proper headers
6. **Forgetting AbortController** → Memory leaks prevention

## Library Dependencies

### Required Dependencies
- `lucide-react`: Modern icon library
- `react-markdown`: Markdown rendering with plugins
- `framer-motion`: Smooth animations
- `remark-gfm`: GitHub-flavored markdown support
- `crypto-js`: Client-side encryption

### Optional Dependencies
- `react-window`: Virtual scrolling for long conversations
- `@types/crypto-js`: TypeScript definitions

## Docusaurus Integration Strategy

1. **Swizzle Layout**: Eject and modify Root.tsx
2. **Client-side only rendering**: Use useIsBrowser hook
3. **Configuration via custom fields**: API endpoint in docusaurus.config.js
4. **Global CSS injection**: Add glassmorphism styles

## Security Considerations

1. **API Key Management**: Environment variables, not hardcoded
2. **Input Validation**: Sanitize all user inputs
3. **XSS Prevention**: Use react-markdown's built-in sanitization
4. **CORS Configuration**: Proper headers on backend
5. **Rate Limiting**: Implement client-side throttling

## Performance Budgets

- **Widget load time**: <100ms
- **First message display**: <500ms
- **Streaming latency**: <100ms per chunk
- **Memory usage**: <50MB for 100 messages
- **Bundle size**: <100KB gzipped

## Error Handling Strategy

1. **Network errors**: Retry with exponential backoff
2. **API errors**: Clear error messages with retry button
3. **SSE disconnects**: Auto-reconnect with visual indicator
4. **Storage errors**: Graceful degradation, continue without persistence

## Testing Strategy

1. **Unit tests**: Hooks and utilities (70%)
2. **Integration tests**: API communication (20%)
3. **E2E tests**: Full user flows (10%)
4. **Performance tests**: Memory usage and render performance

## Mobile Responsiveness

1. **Small screens**: Fullscreen overlay when <400px width
2. **Touch gestures**: Swipe to dismiss, pull to refresh
3. **Virtual keyboard**: Adjust positioning on mobile
4. **Performance**: Reduce animation complexity on low-end devices