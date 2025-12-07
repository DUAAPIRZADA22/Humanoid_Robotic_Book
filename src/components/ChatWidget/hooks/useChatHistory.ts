/**
 * Hook for managing chat history persistence
 * @fileoverview Custom hook for localStorage persistence and history management
 */

import { useCallback, useEffect } from 'react';
import { useChatContext } from '../contexts/ChatContext';
import { loadFromStorage, saveToStorage, clearStorage } from '../utils/storage';

/**
 * Return type for useChatHistory hook
 */
interface UseChatHistoryReturn {
  /** Load chat history from storage */
  loadHistory: () => void;
  /** Save chat history to storage */
  saveHistory: () => void;
  /** Clear all chat history */
  clearHistory: () => void;
  /** Export chat history as JSON */
  exportHistory: () => string;
  /** Import chat history from JSON */
  importHistory: (json: string) => boolean;
}

/**
 * Hook for managing chat history
 * @returns Chat history utilities
 */
export function useChatHistory(): UseChatHistoryReturn {
  const { state, dispatch } = useChatContext();

  /**
   * Load chat history from storage
   */
  const loadHistory = useCallback(() => {
    const savedData = loadFromStorage();
    if (savedData) {
      // Restore messages and settings
      dispatch({
        type: 'RESTORE_STATE',
        payload: {
          messages: savedData.messages,
          settings: savedData.settings,
        },
      } as any);
    }
  }, [dispatch]);

  /**
   * Save chat history to storage
   */
  const saveHistory = useCallback(() => {
    if (state.settings.autoSave) {
      saveToStorage(state.messages, state.settings);
    }
  }, [state.messages, state.settings]);

  /**
   * Clear all chat history
   */
  const clearHistory = useCallback(() => {
    clearStorage();
    dispatch({
      type: 'CLEAR_MESSAGES',
    });
  }, [dispatch]);

  /**
   * Export chat history as JSON
   */
  const exportHistory = useCallback((): string => {
    const exportData = {
      version: '1.0.0',
      exportDate: new Date().toISOString(),
      messages: state.messages,
      settings: state.settings,
    };

    return JSON.stringify(exportData, null, 2);
  }, [state.messages, state.settings]);

  /**
   * Import chat history from JSON
   */
  const importHistory = useCallback((json: string): boolean => {
    try {
      const importData = JSON.parse(json);

      // Validate import data structure
      if (!importData.messages || !Array.isArray(importData.messages)) {
        throw new Error('Invalid import data: missing messages array');
      }

      // Convert timestamps back to Date objects
      const messages = importData.messages.map((msg: any) => ({
        ...msg,
        timestamp: new Date(msg.timestamp),
        isStreaming: false, // Reset streaming state
      }));

      // Update state with imported data
      dispatch({
        type: 'RESTORE_STATE',
        payload: {
          messages,
          settings: { ...state.settings, ...importData.settings },
        },
      } as any);

      return true;
    } catch (error) {
      console.error('Failed to import chat history:', error);
      return false;
    }
  }, [dispatch, state.settings]);

  // Auto-save history when messages change
  useEffect(() => {
    if (state.settings.autoSave && state.messages.length > 0) {
      const timeoutId = setTimeout(() => {
        saveHistory();
      }, 1000); // Debounce save by 1 second

      return () => clearTimeout(timeoutId);
    }
  }, [state.messages, state.settings.autoSave, saveHistory]);

  return {
    loadHistory,
    saveHistory,
    clearHistory,
    exportHistory,
    importHistory,
  };
}