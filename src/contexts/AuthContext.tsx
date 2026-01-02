/**
 * Authentication context provider for User Authentication System.
 * @fileoverview React context for authentication state and methods
 */

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import CryptoJS from 'crypto-js';

/**
 * User data interface
 */
export interface User {
  id: number;
  email: string;
  full_name?: string;
}

/**
 * Auth context value interface
 */
interface AuthContextValue {
  /** Current authenticated user */
  user: User | null;
  /** JWT access token */
  token: string | null;
  /** Loading state */
  loading: boolean;
  /** Error state */
  error: string | null;
  /** Sign in with email and password */
  signIn: (email: string, password: string) => Promise<void>;
  /** Sign up with email, password, and name */
  signUp: (email: string, password: string, fullName: string) => Promise<void>;
  /** Sign out and clear session */
  signOut: () => void;
  /** Update user data */
  updateUser: (user: User) => void;
  /** Clear error */
  clearError: () => void;
}

/**
 * Auth context
 */
const AuthContext = createContext<AuthContextValue | undefined>(undefined);

/**
 * Encryption key for localStorage token storage
 * In production, this should come from environment variable
 */
const ENCRYPTION_KEY = (typeof process !== 'undefined' && process.env?.AUTH_ENCRYPTION_KEY) || 'default-auth-key-change-in-production';

/**
 * Storage keys
 */
const TOKEN_STORAGE_KEY = 'auth_token';
const USER_STORAGE_KEY = 'auth_user';

/**
 * Get API URL from site config or fallback to localhost
 * This is resolved at build time by Docusaurus
 */
function getApiUrl(): string {
  // Try to get from window.DOCUSAURUS_INSTALLED (injected by Docusaurus at build time)
  if (typeof window !== 'undefined' && (window as any).DOCUSAURUS_INSTALLED) {
    const siteConfig = (window as any).DOCUSAURUS_INSTALLED?.siteConfig;
    if (siteConfig?.customFields?.chatApiEndpoint) {
      return siteConfig.customFields.chatApiEndpoint;
    }
  }
  // Fallback to localhost for development
  return 'http://localhost:8000';
}

/**
 * Encrypt data for localStorage
 */
function encryptData(data: string): string {
  return CryptoJS.AES.encrypt(data, ENCRYPTION_KEY).toString();
}

/**
 * Decrypt data from localStorage
 */
function decryptData(encryptedData: string): string {
  try {
    const bytes = CryptoJS.AES.decrypt(encryptedData, ENCRYPTION_KEY);
    return bytes.toString(CryptoJS.enc.Utf8);
  } catch {
    return '';
  }
}

/**
 * Props for AuthProvider
 */
interface AuthProviderProps {
  children: ReactNode;
}

/**
 * Authentication context provider component
 */
export function AuthProvider({ children }: AuthProviderProps): JSX.Element {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  /**
   * Load auth state from localStorage on mount
   */
  useEffect(() => {
    const loadAuthState = () => {
      try {
        // Load and decrypt token
        const storedToken = localStorage.getItem(TOKEN_STORAGE_KEY);
        if (storedToken) {
          const decryptedToken = decryptData(storedToken);
          if (decryptedToken) {
            setToken(decryptedToken);
          }
        }

        // Load user data
        const storedUser = localStorage.getItem(USER_STORAGE_KEY);
        if (storedUser) {
          const parsedUser = JSON.parse(storedUser);
          setUser(parsedUser);
        }
      } catch (err) {
        console.error('Failed to load auth state:', err);
      } finally {
        setLoading(false);
      }
    };

    loadAuthState();
  }, []);

  /**
   * Sign in with email and password
   */
  const signIn = async (email: string, password: string): Promise<void> => {
    setError(null);
    setLoading(true);

    try {
      const response = await fetch(`${getApiUrl()}/api/auth/signin`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Sign in failed');
      }

      setUser(data.user);
      setToken(data.token);

      // Encrypt and store token
      const encryptedToken = encryptData(data.token);
      localStorage.setItem(TOKEN_STORAGE_KEY, encryptedToken);
      localStorage.setItem(USER_STORAGE_KEY, JSON.stringify(data.user));

      // Dispatch event for navbar
      window.dispatchEvent(new CustomEvent('auth:signin'));
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Sign in failed';
      setError(message);
      throw new Error(message);
    } finally {
      setLoading(false);
    }
  };

  /**
   * Sign up with email, password, and name
   */
  const signUp = async (email: string, password: string, fullName: string): Promise<void> => {
    setError(null);
    setLoading(true);

    try {
      const response = await fetch(`${getApiUrl()}/api/auth/register`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email,
          password,
          full_name: fullName || undefined,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Registration failed');
      }

      setUser(data.user);
      setToken(data.token);

      // Encrypt and store token
      const encryptedToken = encryptData(data.token);
      localStorage.setItem(TOKEN_STORAGE_KEY, encryptedToken);
      localStorage.setItem(USER_STORAGE_KEY, JSON.stringify(data.user));

      // Dispatch event for navbar
      window.dispatchEvent(new CustomEvent('auth:signin'));
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Registration failed';
      setError(message);
      throw new Error(message);
    } finally {
      setLoading(false);
    }
  };

  /**
   * Sign out and clear session
   */
  const signOut = (): void => {
    setUser(null);
    setToken(null);
    localStorage.removeItem(TOKEN_STORAGE_KEY);
    localStorage.removeItem(USER_STORAGE_KEY);

    // Dispatch event for navbar
    window.dispatchEvent(new CustomEvent('auth:signout'));
  };

  /**
   * Update user data
   */
  const updateUser = (updatedUser: User): void => {
    setUser(updatedUser);
    localStorage.setItem(USER_STORAGE_KEY, JSON.stringify(updatedUser));
  };

  /**
   * Clear error state
   */
  const clearError = (): void => {
    setError(null);
  };

  const contextValue: AuthContextValue = {
    user,
    token,
    loading,
    error,
    signIn,
    signUp,
    signOut,
    updateUser,
    clearError,
  };

  return (
    <AuthContext.Provider value={contextValue}>
      {children}
    </AuthContext.Provider>
  );
}

/**
 * Hook to use auth context
 * @returns Auth context value
 * @throws Error if used outside AuthProvider
 */
export function useAuth(): AuthContextValue {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
}

export default AuthContext;
