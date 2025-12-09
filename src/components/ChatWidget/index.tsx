/**
 * Main ChatWidget component
 * @fileoverview Root component that orchestrates all chat functionality
 */

import React, { useState, useEffect } from 'react';
import { ChatProvider } from './contexts/ChatContext';
import { useChatStream } from './hooks/useChatStream';
import { useTextSelection } from './hooks/useTextSelection';
import { ChatInterface } from './components/ChatInterface';
import SelectionPopover from './components/SelectionPopover';
import styles from './styles/ChatWidget.module.css';

/**
 * Chat widget props
 */
interface ChatWidgetProps {
  /** Initial open state */
  defaultOpen?: boolean;
  /** Position of the widget */
  position?: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';
  /** Widget size */
  size?: 'small' | 'medium' | 'large';
  /** Additional CSS class name */
  className?: string;
}

/**
 * Internal chat widget component with hooks
 */
function ChatWidgetInner({
  defaultOpen = false,
  position = 'bottom-right',
  size = 'medium',
  className,
}: ChatWidgetProps): JSX.Element {
  const [isOpen, setIsOpen] = useState(defaultOpen);
  const { isStreaming } = useChatStream();
  const { selection, showPopover } = useTextSelection();

  /**
   * Handle custom events
   */
  useEffect(() => {
    // Only run on client side
    if (typeof window === 'undefined') {
      return;
    }

    // Handle clear messages event
    const handleClearMessages = () => {
      window.dispatchEvent(new CustomEvent('chat:clear-messages-context'));
    };

    // Handle retry message event
    const handleRetryMessage = () => {
      window.dispatchEvent(new CustomEvent('chat:retry-message-context'));
    };

    // Handle dismiss error event
    const handleDismissError = () => {
      window.dispatchEvent(new CustomEvent('chat:dismiss-error-context'));
    };

    // Handle ask about selection event
    const handleAskSelection = (event: CustomEvent) => {
      const { text, selectedText } = event.detail;
      window.dispatchEvent(
        new CustomEvent('chat:send-message-context', {
          detail: {
            content: `What does this mean? ${text}`,
            metadata: { selectedText },
          },
        })
      );
    };

    window.addEventListener('chat:clear-messages', handleClearMessages);
    window.addEventListener('chat:retry-message', handleRetryMessage);
    window.addEventListener('chat:dismiss-error', handleDismissError);
    window.addEventListener('chat:ask-selection', handleAskSelection);

    return () => {
      window.removeEventListener('chat:clear-messages', handleClearMessages);
      window.removeEventListener('chat:retry-message', handleRetryMessage);
      window.removeEventListener('chat:dismiss-error', handleDismissError);
      window.removeEventListener('chat:ask-selection', handleAskSelection);
    };
  }, []);

  /**
   * Get position classes
   */
  const getPositionClasses = () => {
    switch (position) {
      case 'bottom-left':
        return styles.bottomLeft;
      case 'top-right':
        return styles.topRight;
      case 'top-left':
        return styles.topLeft;
      default:
        return styles.bottomRight;
    }
  };

  /**
   * Get size classes
   */
  const getSizeClasses = () => {
    switch (size) {
      case 'small':
        return styles.small;
      case 'large':
        return styles.large;
      default:
        return styles.medium;
    }
  };

  return (
    <div
      className={`
        ${styles.chatWidget}
        ${getPositionClasses()}
        ${getSizeClasses()}
        ${className || ''}
      `}
    >
      {/* Chat Interface */}
      <ChatInterface
        isOpen={isOpen}
        onToggle={() => setIsOpen(!isOpen)}
      />

      {/* Text Selection Popover */}
      <SelectionPopover
        selection={selection}
        visible={showPopover}
        onAsk={(text) => {
          // Open widget and send selected text as question
          setIsOpen(true);
          window.dispatchEvent(
            new CustomEvent('chat:ask-selection', {
              detail: { text, selectedText: text },
            })
          );
        }}
      />

      {/* Floating Toggle Button */}
      {!isOpen && (
        <button
          className={`${styles.toggleButton} glass-card ${isStreaming ? styles.streaming : ''}`}
          onClick={() => setIsOpen(true)}
          title="Open chat assistant"
        >
          {isStreaming ? (
            <div className={styles.streamingIcon}>
              <span className={styles.streamingDot} />
              <span className={styles.streamingDot} />
              <span className={styles.streamingDot} />
            </div>
          ) : (
            <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
              <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2z" />
            </svg>
          )}
          <span className={styles.toggleLabel}>Chat</span>
        </button>
      )}
    </div>
  );
}

/**
 * Main ChatWidget component with context provider
 */
export default function ChatWidget(props: ChatWidgetProps): JSX.Element {
  return (
    <ChatProvider>
      <ChatWidgetInner {...props} />
    </ChatProvider>
  );
}

/**
 * Named exports for individual components
 */
export { ChatProvider } from './contexts/ChatContext';
export { useChatContext } from './contexts/ChatContext';
export { useChatStream } from './hooks/useChatStream';
export { useTextSelection } from './hooks/useTextSelection';
export { MessageBubble } from './components/MessageBubble';
export { default as MessageInput } from './components/MessageInput';
export { default as ThinkingIndicator } from './components/ThinkingIndicator';
export { default as SelectionPopover } from './components/SelectionPopover';

/**
 * Export types for external usage
 */
export type {
  ChatMessage,
  ChatState,
  ChatError,
  SessionSettings,
  TextSelection,
} from './types/chat';

export type {
  ApiConfig,
  ChatRequest,
  ChatResponse,
  StreamingChunk,
} from './types/api';