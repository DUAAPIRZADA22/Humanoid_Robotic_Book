/**
 * Thinking indicator component for chat widget
 * @fileoverview Animated loading indicator while AI is thinking
 */

import React from 'react';
import styles from '../styles/ChatWidget.module.css';

interface ThinkingIndicatorProps {
  /** Custom message to display */
  message?: string;
  /** Whether to show animation */
  animated?: boolean;
}

/**
 * Thinking indicator with animated dots
 */
export default function ThinkingIndicator({
  message = 'AI is thinking...',
  animated = true,
}: ThinkingIndicatorProps): JSX.Element {
  return (
    <div className={`${styles.thinkingIndicator} glass-bubble`}>
      <div className={styles.thinkingContent}>
        <div className={styles.thinkingAvatar}>
          <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
            <path d="M9.5,3A6.5,6.5 0 0,1 16,9.5C16,11.11 15.41,12.59 14.44,13.73L14.71,14H15.5L20.5,19L19,20.5L14,15.5V14.71L13.73,14.44C12.59,15.41 11.11,16 9.5,16A6.5,6.5 0 0,1 3,9.5A6.5,6.5 0 0,1 9.5,3M9.5,5C7,5 5,7 5,9.5C5,12 7,14 9.5,14C12,14 14,12 14,9.5C14,7 12,5 9.5,5Z" />
          </svg>
        </div>
        <div className={styles.thinkingText}>
          <span className={styles.message}>{message}</span>
          {animated && (
            <span className={styles.dots}>
              <span className={styles.dot} />
              <span className={styles.dot} />
              <span className={styles.dot} />
            </span>
          )}
        </div>
      </div>
    </div>
  );
}