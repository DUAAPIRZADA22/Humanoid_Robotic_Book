/**
 * Hook for handling text selection and popover positioning
 * @fileoverview Custom hook for text selection detection and UI
 */

import { useState, useCallback, useEffect, useRef } from 'react';
import { TextSelection } from '../types/chat';

interface UseTextSelectionReturn {
  /** Current text selection */
  selection: TextSelection | null;
  /** Whether to show the popover */
  showPopover: boolean;
  /** Clear selection */
  clearSelection: () => void;
}

/**
 * Hook for managing text selection and popover display
 * @returns Text selection utilities
 */
export function useTextSelection(): UseTextSelectionReturn {
  const [selection, setSelection] = useState<TextSelection | null>(null);
  const [showPopover, setShowPopover] = useState(false);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);

  /**
   * Calculate popover position with edge detection
   */
  const calculatePosition = useCallback((selection: Selection): { top: number; left: number } => {
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    // Default position below selection
    let top = rect.bottom + window.scrollY + 8;
    let left = rect.left + window.scrollX;

    // Get viewport dimensions
    const viewportHeight = window.innerHeight;
    const viewportWidth = window.innerWidth;

    // Check if popover would go below viewport
    const popoverHeight = 60; // Approximate popover height
    if (top + popoverHeight - window.scrollY > viewportHeight) {
      // Position above selection instead
      top = rect.top + window.scrollY - popoverHeight - 8;
    }

    // Check if popover would go beyond right edge
    const popoverWidth = 200; // Approximate popover width
    if (left + popoverWidth > viewportWidth) {
      // Align to right edge of selection
      left = rect.right + window.scrollX - popoverWidth;
    }

    // Ensure popover doesn't go beyond left edge
    if (left < 0) {
      left = 10;
    }

    return { top, left };
  }, []);

  /**
   * Handle text selection changes
   */
  const handleSelectionChange = useCallback(() => {
    const userSelection = window.getSelection();

    if (!userSelection || userSelection.isCollapsed) {
      setSelection(null);
      setShowPopover(false);
      return;
    }

    const selectedText = userSelection.toString().trim();

    // Validate selection
    if (selectedText.length === 0) {
      setSelection(null);
      setShowPopover(false);
      return;
    }

    // Check selection length (max 2000 characters)
    if (selectedText.length > 2000) {
      // Truncate with warning - don't modify the selection, just truncate the stored text
      // Modifying the selection can cause IndexSizeError if node length < 2000
    }

    // Get selection range info
    const range = userSelection.getRangeAt(0);
    const position = calculatePosition(userSelection);

    // Create text selection object
    const textSelection: TextSelection = {
      id: `sel_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      text: selectedText.substring(0, 2000),
      range: {
        startOffset: range.startOffset,
        endOffset: range.endOffset,
        collapsed: range.collapsed,
      },
      position: {
        top: position.top,
        left: position.left,
        width: range.getBoundingClientRect().width,
        height: range.getBoundingClientRect().height,
      },
      timestamp: new Date(),
    };

    setSelection(textSelection);

    // Debounce popover display
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
    }

    timeoutRef.current = setTimeout(() => {
      setShowPopover(true);
    }, 300);
  }, [calculatePosition]);

  /**
   * Clear selection and hide popover
   */
  const clearSelection = useCallback(() => {
    setSelection(null);
    setShowPopover(false);

    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
      timeoutRef.current = null;
    }
  }, []);

  /**
   * Handle clicks outside to dismiss popover
   */
  const handleClickOutside = useCallback((event: MouseEvent) => {
    const target = event.target as Element;

    // Check if click is inside popover or selection
    if (
      target.closest('.selection-popover') ||
      target.closest('.chat-widget') ||
      target.closest('.thinking-indicator')
    ) {
      return;
    }

    // Check if clicking on the same selection
    const userSelection = window.getSelection();
    if (userSelection && !userSelection.isCollapsed) {
      const selectedText = userSelection.toString().trim();
      if (selection && selectedText === selection.text) {
        return; // Keep popover open if clicking on same selection
      }
    }

    clearSelection();
  }, [clearSelection, selection]);

  /**
   * Handle keyboard events
   */
  const handleKeyDown = useCallback((event: KeyboardEvent) => {
    // Dismiss on Escape
    if (event.key === 'Escape') {
      clearSelection();
    }
  }, [clearSelection]);

  // Set up event listeners
  useEffect(() => {
    // Add selection change listener
    document.addEventListener('selectionchange', handleSelectionChange);
    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('keyup', handleSelectionChange);

    // Add click outside listener
    document.addEventListener('click', handleClickOutside);

    // Add keyboard listener
    document.addEventListener('keydown', handleKeyDown);

    return () => {
      // Clean up listeners
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('keyup', handleSelectionChange);
      document.removeEventListener('click', handleClickOutside);
      document.removeEventListener('keydown', handleKeyDown);

      // Clear timeout
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
    };
  }, [handleSelectionChange, handleClickOutside, handleKeyDown]);

  // Handle custom events
  useEffect(() => {
    const handleAskSelection = (event: CustomEvent) => {
      clearSelection();
    };

    window.addEventListener('chat:ask-selection', handleAskSelection as EventListener);

    return () => {
      window.removeEventListener('chat:ask-selection', handleAskSelection as EventListener);
    };
  }, [clearSelection]);

  return {
    selection,
    showPopover,
    clearSelection,
  };
}