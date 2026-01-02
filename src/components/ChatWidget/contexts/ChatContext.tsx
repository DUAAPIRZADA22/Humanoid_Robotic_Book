/**
 * Chat context provider
 * @fileoverview React context for chat widget state and actions
 */

import React, { createContext, useContext, useReducer, ReactNode } from 'react';
import { ChatState, ChatMessage, SessionSettings, ChatError } from '../types/chat';
import { chatReducer, initialChatState, chatActions } from './chatReducer';
import { saveToStorage, loadFromStorage } from '../utils/storage';

/**
 * Chat context interface
 */
interface ChatContextValue {
  /** Current chat state */
  state: ChatState;
  /** Dispatch function for state updates */
  dispatch: React.Dispatch<any>;
  /** Send a message to the chat */
  sendMessage: (content: string, metadata?: ChatMessage['metadata']) => Promise<void>;
  /** Clear all messages */
  clearMessages: () => void;
  /** Retry last failed message */
  retryLastMessage: () => void;
  /** Update session settings */
  updateSettings: (settings: Partial<SessionSettings>) => void;
  /** Dismiss current error */
  dismissError: () => void;
}

/**
 * Chat context
 */
const ChatContext = createContext<ChatContextValue | undefined>(undefined);

/**
 * Chat context provider props
 */
interface ChatProviderProps {
  /** Child components */
  children: ReactNode;
  /** Optional initial state override */
  initialState?: Partial<ChatState>;
}

/**
 * Chat context provider component
 */
export function ChatProvider({ children, initialState }: ChatProviderProps): JSX.Element {
  // Load saved state from storage
  const savedState = loadFromStorage();

  // Initialize state with fresh data (no persistence for privacy)
  const [state, dispatch] = useReducer(chatReducer, {
    ...initialChatState,
    ...initialState,
    // Start with empty messages for privacy
    messages: [],
    settings: {
      ...initialChatState.settings,
      autoSave: false, // Disable auto-save for privacy
    },
  });

  // Auto-save to localStorage when messages or settings change
  React.useEffect(() => {
    if (state.settings.autoSave) {
      saveToStorage(state.messages, state.settings);
    }
  }, [state.messages, state.settings]);

  
  /**
   * Send a message to the chat
   */
  const sendMessage = async (content: string, metadata?: ChatMessage['metadata']): Promise<void> => {
    // Validate content
    if (!content.trim()) {
      return;
    }

    // Create user message
    const userMessage: ChatMessage = {
      id: `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      content: content.trim(),
      role: 'user',
      timestamp: new Date(),
      metadata,
    };

    // Add user message to state
    dispatch(chatActions.addMessage(userMessage));

    // Emit custom event for hook to handle API call
    window.dispatchEvent(
      new CustomEvent('chat:send-message', {
        detail: { content: content.trim(), metadata },
      })
    );
  };

  /**
   * Clear all messages
   */
  const clearMessages = (): void => {
    dispatch(chatActions.clearMessages());
  };

  /**
   * Retry the last failed message
   */
  const retryLastMessage = (): void => {
    if (state.messages.length === 0) {
      return;
    }

    // Find the last user message
    const lastUserMessage = [...state.messages]
      .reverse()
      .find(msg => msg.role === 'user');

    if (lastUserMessage) {
      // Remove the last assistant message if it exists
      const filteredMessages = state.messages.slice(0, -1);
      dispatch({
        type: 'SET_MESSAGES',
        payload: filteredMessages,
      } as any);

      // Retry sending the message
      sendMessage(lastUserMessage.content, lastUserMessage.metadata);
    }
  };

  /**
   * Update session settings
   */
  const updateSettings = (settings: Partial<SessionSettings>): void => {
    dispatch(chatActions.updateSettings(settings));
  };

  /**
   * Dismiss current error
   */
  const dismissError = (): void => {
    dispatch(chatActions.setError(null));
  };

  // Handle custom events
  React.useEffect(() => {
    if (typeof window === 'undefined') {
      return;
    }

    // Handle clear messages event
    const handleClearMessages = () => {
      clearMessages();
    };

    // Handle retry message event
    const handleRetryMessage = () => {
      retryLastMessage();
    };

    // Handle dismiss error event
    const handleDismissError = () => {
      dismissError();
    };

    window.addEventListener('chat:clear-messages-context', handleClearMessages);
    window.addEventListener('chat:retry-message-context', handleRetryMessage);
    window.addEventListener('chat:dismiss-error-context', handleDismissError);

    return () => {
      window.removeEventListener('chat:clear-messages-context', handleClearMessages);
      window.removeEventListener('chat:retry-message-context', handleRetryMessage);
      window.removeEventListener('chat:dismiss-error-context', handleDismissError);
    };
  }, [clearMessages, retryLastMessage, dismissError]);

  const contextValue: ChatContextValue = {
    state,
    dispatch,
    sendMessage,
    clearMessages,
    retryLastMessage,
    updateSettings,
    dismissError,
  };

  return (
    <ChatContext.Provider value={contextValue}>
      {children}
    </ChatContext.Provider>
  );
}

/**
 * Hook to use chat context
 * @returns Chat context value
 */
export function useChatContext(): ChatContextValue {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within a ChatProvider');
  }
  return context;
}