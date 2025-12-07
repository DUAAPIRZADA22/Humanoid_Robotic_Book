# Quick Start: Chat Widget Frontend

**Feature**: 003-chat-widget-frontend
**Date**: 2025-12-07

## Prerequisites

1. Node.js 16+ installed
2. Docusaurus 3.x project set up
3. Backend API endpoint available
4. API key for authentication

## Step 1: Install Dependencies

```bash
# Required dependencies
npm install lucide-react react-markdown framer-motion remark-gfm crypto-js

# Optional dependencies for performance
npm install react-window @types/crypto-js
```

## Step 2: Configure Docusaurus

Update `docusaurus.config.js`:

```javascript
module.exports = {
  // ... existing config
  themeConfig: {
    // ... existing theme config
  },
  customFields: {
    chatApiEndpoint: 'http://localhost:7860/api/v1', // Development
    // chatApiEndpoint: 'https://api.example.com/api/v1', // Production
    chatApiKey: process.env.CHAT_API_KEY || 'your-api-key'
  }
};
```

Create `.env` file:

```env
CHAT_API_KEY=your-actual-api-key-here
```

## Step 3: Create Directory Structure

```bash
mkdir -p src/components/ChatWidget/{components,hooks,contexts,utils,styles}
mkdir -p src/theme
```

## Step 4: Implement Core Components

### 1. Create the main ChatWidget component

`src/components/ChatWidget/index.tsx`:

```typescript
import React from 'react';
import { useChatState } from '../hooks/useChatState';
import { useDocusaurusContext } from '@docusaurus/useDocusaurusContext';

export const ChatWidget: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();
  const { messages, sendMessage, isLoading, error } = useChatState(
    siteConfig.customFields.chatApiEndpoint as string,
    siteConfig.customFields.chatApiKey as string
  );

  // Component implementation
  return (
    <div className="fixed bottom-4 right-4 w-96 h-[600px] glass-card rounded-2xl">
      {/* Widget content */}
    </div>
  );
};
```

### 2. Create the chat hook

`src/components/ChatWidget/hooks/useChatState.ts`:

```typescript
import { useReducer, useCallback } from 'react';

// Reducer and hook implementation
export const useChatState = (endpoint: string, apiKey: string) => {
  const [state, dispatch] = useReducer(chatReducer, initialState);

  const sendMessage = useCallback(async (content: string) => {
    // Implementation
  }, [endpoint, apiKey]);

  return {
    messages: state.messages,
    sendMessage,
    isLoading: state.isLoading,
    error: state.error
  };
};
```

### 3. Create glassmorphism styles

`src/components/ChatWidget/styles/glassmorphism.css`:

```css
.glass-card {
  background: rgba(255, 255, 255, 0.05);
  backdrop-filter: blur(20px) saturate(180%);
  border: 1px solid rgba(255, 255, 255, 0.125);
  box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);
}

/* Add to src/css/custom.css */
@import '../components/ChatWidget/styles/glassmorphism.css';
```

## Step 5: Mount Widget Globally

1. Swizzle the Root component:

```bash
npm run swizzle @docusaurus/theme-classic Layout -- --eject
```

2. Update `src/theme/Layout.tsx`:

```typescript
import React from 'react';
import Layout from '@theme-original/Layout';
import { ChatWidget } from '@site/src/components/ChatWidget';

export default function LayoutWrapper(props: any): JSX.Element {
  return (
    <>
      <Layout {...props} />
      <ChatWidget />
    </>
  );
}
```

## Step 6: Test the Implementation

1. Start your development server:

```bash
npm start
```

2. Open your Docusaurus site
3. The chat widget should appear in the bottom-right corner
4. Try sending a message
5. Test text selection feature

## Common Issues & Solutions

### Widget not appearing
- Check browser console for errors
- Verify API endpoint is configured correctly
- Ensure CSS is imported properly

### SSE connection failing
- Check CORS settings on backend
- Verify API key is valid
- Check network tab in developer tools

### Glassmorphism not working
- Ensure backdrop-filter is supported (modern browsers)
- Check CSS imports
- Verify Tailwind configuration

### Text selection not working
- Check if `user-select: none` is applied to parent elements
- Verify event listeners are properly attached
- Test with different browsers

## Performance Tips

1. **Message List Optimization**:
   ```typescript
   const MessageBubble = React.memo(({ message }) => {
     // Component implementation
   });
   ```

2. **Virtual Scrolling** (for >50 messages):
   ```typescript
   import { FixedSizeList as List } from 'react-window';
   ```

3. **Debounce expensive operations**:
   ```typescript
   const debouncedSave = useCallback(
     debounce((messages) => saveToStorage(messages), 500),
     []
   );
   ```

## Security Best Practices

1. **Never expose API keys in client code**:
   ```javascript
   // Bad
   const apiKey = 'hardcoded-key';

   // Good
   const apiKey = process.env.CHAT_API_KEY;
   ```

2. **Sanitize all inputs**:
   ```typescript
   import DOMPurify from 'dompurify';

   const sanitizedContent = DOMPurify.sanitize(content);
   ```

3. **Use HTTPS in production**:
   ```javascript
   const endpoint = isDevelopment
     ? 'http://localhost:7860'
     : 'https://api.example.com';
   ```

## Next Steps

1. Customize the widget appearance
2. Add error boundaries
3. Implement analytics tracking
4. Add accessibility features
5. Test on mobile devices
6. Deploy to production

## Need Help?

- Check the implementation plan for detailed patterns
- Review the API contract documentation
- Look at the data model for entity definitions
- Check the research findings for best practices