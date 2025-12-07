/**
 * Message input component for chat widget
 * @fileoverview Input field with send button and keyboard shortcuts
 */

import React, { useState, useRef, useEffect } from 'react';
import { useChatContext } from '../contexts/ChatContext';
import styles from '../styles/ChatWidget.module.css';

interface MessageInputProps {
  /** Whether input is disabled */
  disabled?: boolean;
  /** Input placeholder text */
  placeholder?: string;
  /** Maximum message length */
  maxLength?: number;
  /** Additional CSS class name */
  className?: string;
}

/**
 * Message input component with keyboard shortcuts
 */
export default function MessageInput({
  disabled = false,
  placeholder = 'Type your message...',
  maxLength = 10000,
  className,
}: MessageInputProps): JSX.Element {
  const { state, sendMessage } = useChatContext();
  const [inputValue, setInputValue] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  /**
   * Handle form submission
   */
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || disabled || state.isLoading || state.isStreaming) {
      return;
    }

    sendMessage(inputValue.trim());
    setInputValue('');
  };

  /**
   * Handle keyboard shortcuts
   */
  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      // Send message on Enter (without Shift)
      e.preventDefault();
      handleSubmit(e);
    }
  };

  /**
   * Auto-resize textarea
   */
  useEffect(() => {
    const textarea = textareaRef.current;
    if (textarea) {
      // Reset height to auto to get the correct scrollHeight
      textarea.style.height = 'auto';
      // Set height to scrollHeight (capped at max height)
      const maxHeight = 120; // Max height in pixels
      const newHeight = Math.min(textarea.scrollHeight, maxHeight);
      textarea.style.height = `${newHeight}px`;
    }
  }, [inputValue]);

  /**
   * Focus input when not disabled
   */
  useEffect(() => {
    if (!disabled && textareaRef.current) {
      textareaRef.current.focus();
    }
  }, [disabled]);

  const characterCount = inputValue.length;
  const isNearLimit = maxLength > 0 && characterCount > maxLength * 0.9;
  const isAtLimit = maxLength > 0 && characterCount >= maxLength;

  return (
    <form
      className={`${styles.messageInput} ${className || ''}`}
      onSubmit={handleSubmit}
    >
      <div className={styles.inputContainer}>
        <textarea
          ref={textareaRef}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={placeholder}
          disabled={disabled}
          maxLength={maxLength}
          className={`${styles.textarea} glass-input`}
          rows={1}
          style={{
            resize: 'none',
            overflowY: 'auto',
          }}
        />

        {/* Character count indicator */}
        {maxLength > 0 && (isNearLimit || isAtLimit) && (
          <div className={`${styles.characterCount} ${isAtLimit ? styles.limit : ''}`}>
            {characterCount}/{maxLength}
          </div>
        )}
      </div>

      <div className={styles.inputActions}>
        {/* Send button */}
        <button
          type="submit"
          disabled={
            disabled ||
            !inputValue.trim() ||
            state.isLoading ||
            state.isStreaming ||
            isAtLimit
          }
          className={`${styles.sendButton} glass-button ${
            (!inputValue.trim() || state.isLoading || state.isStreaming || isAtLimit) ? styles.disabled : ''
          }`}
          title="Send message (Enter)"
        >
          {state.isLoading || state.isStreaming ? (
            <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor" className={styles.spinner}>
              <path d="M12,1A11,11,0,1,0,23,12,11,11,0,0,0,12,1Zm0,19a8,8,0,1,1,8-8A8,8,0,0,1,12,20Z" opacity=".25"/>
              <path d="M10.14,8.86a2,2,0,0,0,0,2.82l2.5,2.5a2,2,0,0,0,2.82,0l2.5-2.5a2,2,0,0,0,0-2.82L15.46,6.36A2,2,0,0,0,12.64,6.36L10.14,8.86Z"/>
            </svg>
          ) : (
            <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
              <path d="M2,21L23,12L2,3V10L17,12L2,14V21Z" />
            </svg>
          )}
        </button>
      </div>

      {/* Input hints */}
      <div className={styles.inputHints}>
        <span className={styles.hint}>Press Enter to send, Shift+Enter for new line</span>
      </div>
    </form>
  );
}