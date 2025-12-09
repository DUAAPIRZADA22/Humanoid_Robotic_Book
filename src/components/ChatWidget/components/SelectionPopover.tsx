/**
 * Selection popover component for quick questions
 * @fileoverview Popover UI for asking about selected text
 */

import React, { useEffect, useRef } from 'react';
import { TextSelection } from '../types/chat';
import styles from '../styles/ChatWidget.module.css';

interface SelectionPopoverProps {
  /** Current text selection */
  selection: TextSelection | null;
  /** Whether popover is visible */
  visible: boolean;
  /** Callback when user asks about selection */
  onAsk: (text: string) => void;
}

/**
 * Selection popover component
 */
export default function SelectionPopover({
  selection,
  visible,
  onAsk,
}: SelectionPopoverProps): JSX.Element {
  const popoverRef = useRef<HTMLDivElement>(null);

  /**
   * Handle click outside to close popover
   */
  useEffect(() => {
    // Only run on client side
    if (typeof window === 'undefined') {
      return;
    }

    const handleClickOutside = (event: MouseEvent) => {
      if (
        popoverRef.current &&
        !popoverRef.current.contains(event.target as Node)
      ) {
        // Let parent handle outside clicks
        return;
      }
    };

    if (visible) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [visible]);

  /**
   * Position popover using the stored position
   */
  const getPopoverStyle = (): React.CSSProperties => {
    if (!selection) {
      return { display: 'none' };
    }

    return {
      position: 'fixed',
      top: `${selection.position.top}px`,
      left: `${selection.position.left}px`,
      zIndex: 1001,
      opacity: visible ? 1 : 0,
      transform: visible ? 'translateY(0)' : 'translateY(-10px)',
      transition: 'all 0.2s ease',
      pointerEvents: visible ? 'auto' : 'none',
    };
  };

  /**
   * Truncate selected text for display
   */
  const truncateText = (text: string, maxLength: number = 50): string => {
    if (text.length <= maxLength) {
      return text;
    }
    return `${text.substring(0, maxLength)}...`;
  };

  /**
   * Handle ask button click
   */
  const handleAsk = () => {
    if (selection) {
      onAsk(selection.text);
    }
  };

  /**
   * Handle keyboard interaction
   */
  const handleKeyDown = (event: React.KeyboardEvent) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      handleAsk();
    } else if (event.key === 'Escape') {
      // Let parent handle escape
      return;
    }
  };

  if (!selection) {
    return null;
  }

  return (
    <div
      ref={popoverRef}
      className={`${styles.selectionPopover} selection-popover glass-popover`}
      style={getPopoverStyle()}
      onKeyDown={handleKeyDown}
      role="tooltip"
      aria-label="Ask about selected text"
    >
      <div className={styles.popoverContent}>
        <div className={styles.selectedTextPreview}>
          <span className={styles.previewIcon}>
            <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
              <path d="M14,17H17L19,13V7H13V13H16M6,17H9L11,13V7H5V13H8L6,17Z" />
            </svg>
          </span>
          <span className={styles.previewText}>
            {truncateText(selection.text)}
          </span>
        </div>

        <button
          className={`${styles.askButton} glass-button`}
          onClick={handleAsk}
          title="Ask AI about this text"
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor" style={{ marginRight: '6px' }}>
            <path d="M9.5,3A6.5,6.5 0 0,1 16,9.5C16,11.11 15.41,12.59 14.44,13.73L14.71,14H15.5L20.5,19L19,20.5L14,15.5V14.71L13.73,14.44C12.59,15.41 11.11,16 9.5,16A6.5,6.5 0 0,1 3,9.5A6.5,6.5 0 0,1 9.5,3M9.5,5C7,5 5,7 5,9.5C5,12 7,14 9.5,14C12,14 14,12 14,9.5C14,7 12,5 9.5,5Z" />
          </svg>
          Ask AI
        </button>
      </div>

      {/* Warning for truncated text */}
      {selection.text.length > 50 && (
        <div className={styles.truncationWarning}>
          <span className={styles.warningIcon}>⚠</span>
          <span className={styles.warningText}>
            First 50 characters shown. Full text will be used.
          </span>
        </div>
      )}

      {/* Warning for long text selection */}
      {selection.text.length > 1000 && (
        <div className={styles.truncationWarning}>
          <span className={styles.warningIcon}>📝</span>
          <span className={styles.warningText}>
            Long text selected ({selection.text.length} characters). Consider selecting a smaller portion for better results.
          </span>
        </div>
      )}
    </div>
  );
}