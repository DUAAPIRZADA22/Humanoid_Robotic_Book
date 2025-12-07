/**
 * Chat reducer for state management
 * @fileoverview Reducer function for chat state using useReducer pattern
 */

import { ChatState, ChatAction, ChatMessage, SessionSettings } from '../types/chat';

/**
 * Initial chat state
 */
export const initialChatState: ChatState = {
  messages: [],
  isLoading: false,
  isStreaming: false,
  error: null,
  settings: {
    maxMessages: 100,
    autoSave: true,
    streamingEnabled: true,
  },
};

/**
 * Chat reducer function
 * @param state - Current chat state
 * @param action - Action to dispatch
 * @returns New chat state
 */
export function chatReducer(state: ChatState, action: ChatAction): ChatState {
  switch (action.type) {
    case 'ADD_MESSAGE': {
      const newMessages = [...state.messages, action.payload];

      // Limit messages to maxMessages setting
      const limitedMessages = newMessages.slice(-state.settings.maxMessages);

      return {
        ...state,
        messages: limitedMessages,
        isLoading: false,
        error: null,
      };
    }

    case 'START_STREAM': {
      // Create a new streaming message
      const streamingMessage: ChatMessage = {
        id: action.payload,
        content: '',
        role: 'assistant',
        timestamp: new Date(),
        isStreaming: true,
      };

      return {
        ...state,
        messages: [...state.messages, streamingMessage],
        isStreaming: true,
        streamingId: action.payload,
        isLoading: false,
        error: null,
      };
    }

    case 'APPEND_STREAM': {
      if (!state.streamingId) {
        return state;
      }

      const updatedMessages = state.messages.map(msg =>
        msg.id === state.streamingId && msg.isStreaming
          ? { ...msg, content: msg.content + action.payload }
          : msg
      );

      return {
        ...state,
        messages: updatedMessages,
      };
    }

    case 'END_STREAM': {
      if (!state.streamingId) {
        return state;
      }

      const updatedMessages = state.messages.map(msg =>
        msg.id === state.streamingId && msg.isStreaming
          ? { ...msg, isStreaming: false }
          : msg
      );

      return {
        ...state,
        messages: updatedMessages,
        isStreaming: false,
        streamingId: undefined,
      };
    }

    case 'SET_LOADING': {
      return {
        ...state,
        isLoading: action.payload,
        error: action.payload ? null : state.error,
      };
    }

    case 'SET_ERROR': {
      return {
        ...state,
        error: action.payload,
        isLoading: false,
        isStreaming: false,
        streamingId: undefined,
      };
    }

    case 'CLEAR_MESSAGES': {
      return {
        ...state,
        messages: [],
        error: null,
      };
    }

    case 'UPDATE_SETTINGS': {
      const newSettings = { ...state.settings, ...action.payload };

      // Apply new maxMessages limit if changed
      let messages = state.messages;
      if (action.payload.maxMessages !== undefined) {
        messages = messages.slice(-action.payload.maxMessages);
      }

      return {
        ...state,
        messages,
        settings: newSettings,
      };
    }

    default: {
      // Type safety: exhaustive check
      const _exhaustiveCheck: never = action;
      return state;
    }
  }
}

/**
 * Action creators for common operations
 */
export const chatActions = {
  /**
   * Add a message to the chat
   */
  addMessage: (message: ChatMessage): ChatAction => ({
    type: 'ADD_MESSAGE',
    payload: message,
  }),

  /**
   * Start streaming a response
   */
  startStream: (messageId: string): ChatAction => ({
    type: 'START_STREAM',
    payload: messageId,
  }),

  /**
   * Append content to streaming message
   */
  appendStream: (content: string): ChatAction => ({
    type: 'APPEND_STREAM',
    payload: content,
  }),

  /**
   * End streaming
   */
  endStream: (): ChatAction => ({
    type: 'END_STREAM',
  }),

  /**
   * Set loading state
   */
  setLoading: (loading: boolean): ChatAction => ({
    type: 'SET_LOADING',
    payload: loading,
  }),

  /**
   * Set error state
   */
  setError: (error: ChatError | null): ChatAction => ({
    type: 'SET_ERROR',
    payload: error,
  }),

  /**
   * Clear all messages
   */
  clearMessages: (): ChatAction => ({
    type: 'CLEAR_MESSAGES',
  }),

  /**
   * Update session settings
   */
  updateSettings: (settings: Partial<SessionSettings>): ChatAction => ({
    type: 'UPDATE_SETTINGS',
    payload: settings,
  }),
};