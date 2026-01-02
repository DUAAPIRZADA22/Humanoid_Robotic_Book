/**
 * Main chat interface component
 * @fileoverview Chat widget UI with messages list and input
 */

import React, { useState, useRef, useEffect } from 'react';
import { useChatContext } from '../contexts/ChatContext';
import { MessageBubble } from './MessageBubble';
import MessageInput from './MessageInput';
import ThinkingIndicator from './ThinkingIndicator';
import styles from '../styles/ChatWidget.module.css';

interface ChatInterfaceProps {
  /** Widget visibility state */
  isOpen: boolean;
  /** Toggle widget visibility */
  onToggle: () => void;
  /** Additional CSS class name */
  className?: string;
}

/**
 * Chat interface component
 */
export function ChatInterface({ isOpen, onToggle, className }: ChatInterfaceProps): JSX.Element {
  const { state } = useChatContext();
  const [isScrolledToBottom, setIsScrolledToBottom] = useState(true);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const messagesContainerRef = useRef<HTMLDivElement>(null);

  /**
   * Auto-scroll to bottom when new messages arrive
   */
  const scrollToBottom = useCallback(() => {
    if (messagesEndRef.current && isScrolledToBottom) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [isScrolledToBottom]);

  /**
   * Handle scroll events to detect if user is at bottom
   */
  const handleScroll = useCallback(() => {
    if (messagesContainerRef.current) {
      const { scrollTop, scrollHeight, clientHeight } = messagesContainerRef.current;
      const isAtBottom = scrollTop + clientHeight >= scrollHeight - 50;
      setIsScrolledToBottom(isAtBottom);
    }
  }, []);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [state.messages, scrollToBottom]);

  /**
   * Render error message with retry button
   */
  const renderError = () => {
    if (!state.error) return null;

    return (
      <div className={`${styles.error} glass-card`}>
        <div className={styles.errorContent}>
          <span className={styles.errorMessage}>{state.error.message}</span>
          {state.error.retryable && (
            <button
              className={`${styles.retryButton} glass-button`}
              onClick={() => {
                // Trigger retry through context
                if (typeof window !== 'undefined') {
                  window.dispatchEvent(new CustomEvent('chat:retry-message'));
                }
              }}
            >
              Retry
            </button>
          )}
          <button
            className={`${styles.dismissButton} glass-button`}
            onClick={() => {
              // Dismiss error through context
              if (typeof window !== 'undefined') {
                window.dispatchEvent(new CustomEvent('chat:dismiss-error'));
              }
            }}
          >
            Dismiss
          </button>
        </div>
      </div>
    );
  };

  /**
   * Render empty state when no messages
   */
  const renderEmptyState = () => {
    if (state.messages.length > 0) return null;

    return (
      <div className={styles.emptyState}>
        <div className={styles.emptyStateContent}>
          <h3 className={styles.emptyStateTitle}>Welcome to Physical AI Assistant</h3>
          <p className={styles.emptyStateText}>
            Ask me anything about Physical AI & Humanoid Robotics. You can also select text on the page for contextual questions.
          </p>
        </div>
      </div>
    );
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div className={`${styles.chatInterface} ${className || ''} glass-card`}>
      {/* Header */}
      <div className={`${styles.header} glass-card-dark`}>
        <div className={styles.headerContent}>
          <div className={styles.headerLeft}>
            <div className={`${styles.statusIndicator} ${state.isStreaming ? styles.streaming : ''}`} />
            <h2 className={styles.title}>Physical AI Assistant</h2>
          </div>
          <div className={styles.headerRight}>
            <button
              className={`${styles.headerButton} glass-button`}
              onClick={() => {
                if (typeof window !== 'undefined') {
                  window.dispatchEvent(new CustomEvent('chat:clear-messages'));
                }
              }}
              title="Clear conversation"
            >
              Clear
            </button>
            <button
              className={`${styles.closeButton} glass-button`}
              onClick={onToggle}
              title="Close chat"
            >
              ×
            </button>
          </div>
        </div>
      </div>

      {/* Messages Container */}
      <div
        ref={messagesContainerRef}
        className={`${styles.messagesContainer} glass-scrollbar`}
        onScroll={handleScroll}
      >
        {/* Error Display */}
        {renderError()}

        {/* Messages List */}
        <div className={styles.messagesList}>
          {renderEmptyState()}
          {state.messages.map((message, index) => (
            <MessageBubble
              key={message.id}
              message={message}
              isStreaming={message.isStreaming}
              isLast={index === state.messages.length - 1}
            />
          ))}
        </div>

        {/* Scroll to bottom indicator */}
        {!isScrolledToBottom && (
          <button
            className={`${styles.scrollToBottom} glass-button`}
            onClick={() => {
              setIsScrolledToBottom(true);
              scrollToBottom();
            }}
          >
            ↓ New messages
          </button>
        )}

        {/* Thinking Indicator */}
        {state.isLoading && !state.isStreaming && <ThinkingIndicator />}

        {/* Invisible element for scroll targeting */}
        <div ref={messagesEndRef} />
      </div>

      {/* Message Input */}
      <div className={`${styles.inputContainer} glass-card-dark`}>
        <MessageInput
          disabled={state.isLoading || state.isStreaming}
          placeholder={
            state.error
              ? 'Try sending your message again...'
              : 'Ask about Physical AI & Humanoid Robotics...'
          }
        />
      </div>
    </div>
  );
}

// Import useCallback for the nested functions
import { useCallback } from 'react';