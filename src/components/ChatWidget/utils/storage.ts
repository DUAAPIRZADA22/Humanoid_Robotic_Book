/**
 * Encrypted localStorage utility for chat widget
 * @fileoverview Secure storage implementation using AES-256 encryption
 */

import CryptoJS from 'crypto-js';
import { StorageEntry, ChatMessage, SessionSettings } from '../types/chat';

const STORAGE_KEY = 'chatWidgetData';
const ENCRYPTION_KEY = 'chat-widget-encryption-key';
const STORAGE_VERSION = '1.0.0';

/**
 * Default session settings
 */
const DEFAULT_SETTINGS: SessionSettings = {
  maxMessages: 100,
  autoSave: true,
  streamingEnabled: true,
};

/**
 * Encrypt data using AES-256
 * @param data - Data to encrypt
 * @returns Encrypted string
 */
function encrypt(data: string): string {
  return CryptoJS.AES.encrypt(data, ENCRYPTION_KEY).toString();
}

/**
 * Decrypt data using AES-256
 * @param encryptedData - Encrypted data
 * @returns Decrypted string
 */
function decrypt(encryptedData: string): string {
  const bytes = CryptoJS.AES.decrypt(encryptedData, ENCRYPTION_KEY);
  return bytes.toString(CryptoJS.enc.Utf8);
}

/**
 * Save chat data to encrypted localStorage
 * @param messages - Messages to save
 * @param settings - Session settings to save
 */
export function saveToStorage(
  messages: ChatMessage[],
  settings: SessionSettings = DEFAULT_SETTINGS
): void {
  try {
    // Limit messages to maxMessages setting
    const limitedMessages = messages.slice(-settings.maxMessages);

    const storageEntry: StorageEntry = {
      version: STORAGE_VERSION,
      timestamp: Date.now(),
      data: {
        messages: limitedMessages.map(msg => ({
          ...msg,
          timestamp: new Date(msg.timestamp), // Ensure Date serialization
        })),
        settings,
      },
    };

    const serialized = JSON.stringify(storageEntry);
    const encrypted = encrypt(serialized);
    localStorage.setItem(STORAGE_KEY, encrypted);
  } catch (error) {
    console.error('Failed to save to storage:', error);
    // Silently fail - widget should continue without persistence
  }
}

/**
 * Load chat data from encrypted localStorage
 * @returns Storage entry or null if not found/invalid
 */
export function loadFromStorage(): {
  messages: ChatMessage[];
  settings: SessionSettings;
} | null {
  try {
    const encrypted = localStorage.getItem(STORAGE_KEY);
    if (!encrypted) {
      return null;
    }

    const decrypted = decrypt(encrypted);
    if (!decrypted) {
      return null;
    }

    const storageEntry: StorageEntry = JSON.parse(decrypted);

    // Validate version
    if (storageEntry.version !== STORAGE_VERSION) {
      console.warn('Storage version mismatch, clearing old data');
      clearStorage();
      return null;
    }

    // Convert timestamps back to Date objects
    const messages = storageEntry.data.messages.map(msg => ({
      ...msg,
      timestamp: new Date(msg.timestamp),
    }));

    return {
      messages,
      settings: { ...DEFAULT_SETTINGS, ...storageEntry.data.settings },
    };
  } catch (error) {
    console.error('Failed to load from storage:', error);
    return null;
  }
}

/**
 * Clear all chat data from localStorage
 */
export function clearStorage(): void {
  try {
    localStorage.removeItem(STORAGE_KEY);
  } catch (error) {
    console.error('Failed to clear storage:', error);
  }
}

/**
 * Check if localStorage is available
 * @returns True if localStorage is accessible
 */
export function isStorageAvailable(): boolean {
  try {
    const test = 'test';
    localStorage.setItem(test, test);
    localStorage.removeItem(test);
    return true;
  } catch {
    return false;
  }
}

/**
 * Get storage size in bytes
 * @returns Storage size in bytes
 */
export function getStorageSize(): number {
  try {
    const encrypted = localStorage.getItem(STORAGE_KEY);
    return encrypted ? new Blob([encrypted]).size : 0;
  } catch {
    return 0;
  }
}

/**
 * Save individual settings to storage
 * @param settings - Settings to save
 */
export function saveSettings(settings: Partial<SessionSettings>): void {
  const current = loadFromStorage();
  if (current) {
    saveToStorage(current.messages, { ...current.settings, ...settings });
  } else {
    saveToStorage([], { ...DEFAULT_SETTINGS, ...settings });
  }
}