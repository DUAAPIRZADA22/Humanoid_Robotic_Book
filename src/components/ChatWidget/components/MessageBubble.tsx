/**
 * Message bubble component for displaying chat messages
 * @fileoverview Individual message display with markdown rendering
 */

import React, { memo } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { ChatMessage } from '../types/chat';
import styles from '../styles/ChatWidget.module.css';

interface MessageBubbleProps {
  /** Message to display */
  message: ChatMessage;
  /** Whether message is currently streaming */
  isStreaming?: boolean;
  /** Whether this is the last message in the list */
  isLast?: boolean;
}

/**
 * Message bubble component with memoization
 */
export const MessageBubble = memo<MessageBubbleProps>(function MessageBubble({
  message,
  isStreaming = false,
  isLast = false,
}): JSX.Element {
  const isUser = message.role === 'user';
  const isAssistant = message.role === 'assistant';

  /**
   * Format timestamp for display
   */
  const formatTime = (date: Date): string => {
    return date.toLocaleTimeString(undefined, {
      hour: '2-digit',
      minute: '2-digit',
    });
  };

  /**
   * Render message content with markdown support
   */
  const renderContent = () => {
    if (isUser) {
      // User messages are plain text
      return <p className={styles.messageText}>{message.content}</p>;
    }

    // Assistant messages support markdown
    return (
      <ReactMarkdown
        remarkPlugins={[remarkGfm]}
        className={styles.markdownContent}
        components={{
          // Custom component for code blocks
          code: ({ node, inline, className, children, ...props }) => {
            const match = /language-(\w+)/.exec(className || '');
            return !inline && match ? (
              <pre className={styles.codeBlock}>
                <code className={className} {...props}>
                  {children}
                </code>
              </pre>
            ) : (
              <code className={styles.inlineCode} {...props}>
                {children}
              </code>
            );
          },
          // Custom component for links
          a: ({ href, children }) => (
            <a
              href={href}
              target="_blank"
              rel="noopener noreferrer"
              className={styles.messageLink}
            >
              {children}
            </a>
          ),
        }}
      >
        {message.content}
      </ReactMarkdown>
    );
  };

  return (
    <div
      className={`
        ${styles.messageBubble}
        ${isUser ? styles.userMessage : styles.assistantMessage}
        ${isStreaming ? styles.streaming : ''}
        glass-bubble
      `}
    >
      {/* Message header with avatar and role */}
      <div className={styles.messageHeader}>
        <div className={styles.messageAvatar}>
          {isUser ? (
            <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
              <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
            </svg>
          ) : (
            <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
              <path d="M9.5,3A6.5,6.5 0 0,1 16,9.5C16,11.11 15.41,12.59 14.44,13.73L14.71,14H15.5L20.5,19L19,20.5L14,15.5V14.71L13.73,14.44C12.59,15.41 11.11,16 9.5,16A6.5,6.5 0 0,1 3,9.5A6.5,6.5 0 0,1 9.5,3M9.5,5C7,5 5,7 5,9.5C5,12 7,14 9.5,14C12,14 14,12 14,9.5C14,7 12,5 9.5,5Z" />
            </svg>
          )}
        </div>
        <div className={styles.messageMeta}>
          <span className={styles.messageRole}>
            {isUser ? 'You' : 'AI Assistant'}
          </span>
          <span className={styles.messageTime}>
            {formatTime(message.timestamp)}
          </span>
        </div>
      </div>

      {/* Message content */}
      <div className={styles.messageContent}>
        {renderContent()}
        {isStreaming && (
          <span className={styles.streamingCursor}>|</span>
        )}
      </div>

      {/* Message metadata */}
      {message.metadata && (
        <div className={styles.messageMetadata}>
          {message.metadata.selectedText && (
            <div className={styles.selectedTextContext}>
              <span className={styles.contextLabel}>About:</span>
              <span className={styles.contextText}>
                "{message.metadata.selectedText}"
              </span>
            </div>
          )}
        </div>
      )}

      {/* Streaming indicator */}
      {isStreaming && isLast && (
        <div className={styles.streamingIndicator}>
          <span className={styles.streamingDot} />
          <span className={styles.streamingDot} />
          <span className={styles.streamingDot} />
        </div>
      )}
    </div>
  );
});