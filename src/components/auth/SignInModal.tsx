/**
 * Sign In Modal Component
 * @fileoverview Modal for user sign in with email and password
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import styles from './AuthModal.module.css';

/**
 * Props for SignInModal
 */
export interface SignInModalProps {
  /** Callback when modal should close */
  onClose: () => void;
  /** Callback to switch to sign up modal */
  onSwitchToSignUp: () => void;
  /** Callback to switch to password reset */
  onSwitchToReset?: () => void;
}

/**
 * Sign In Modal Component
 */
export function SignInModal({
  onClose,
  onSwitchToSignUp,
  onSwitchToReset,
}: SignInModalProps): JSX.Element {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const { signIn, clearError } = useAuth();

  // Clear error when input changes
  useEffect(() => {
    if (email || password) {
      setError('');
    }
  }, [email, password]);

  /**
   * Handle form submission
   */
  const handleSubmit = async (e: React.FormEvent): Promise<void> => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      await signIn(email, password);
      clearError();
      onClose();
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Sign in failed';
      setError(message);
    } finally {
      setLoading(false);
    }
  };

  /**
   * Handle close on escape key
   */
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent): void => {
      if (e.key === 'Escape' && !loading) {
        onClose();
      }
    };

    window.addEventListener('keydown', handleEscape);
    return () => window.removeEventListener('keydown', handleEscape);
  }, [onClose, loading]);

  return (
    <div className={styles.modalOverlay} onClick={loading ? undefined : onClose}>
      <div
        className={`glass-card ${styles.modalContent}`}
        onClick={(e) => e.stopPropagation()}
      >
        {/* Header */}
        <div className={styles.modalHeader}>
          <h2 className={styles.modalTitle}>Sign In</h2>
          <button
            className={styles.closeButton}
            onClick={onClose}
            disabled={loading}
            aria-label="Close"
          >
            ×
          </button>
        </div>

        {/* Form */}
        <form onSubmit={handleSubmit} className={styles.form}>
          {/* Error Message */}
          {error && (
            <div className={styles.errorMessage} role="alert">
              {error}
            </div>
          )}

          {/* Email Field */}
          <div className={styles.fieldGroup}>
            <label htmlFor="signin-email" className={styles.label}>
              Email
            </label>
            <input
              id="signin-email"
              type="email"
              className={styles.input}
              placeholder="you@example.com"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              disabled={loading}
              required
              autoComplete="email"
            />
          </div>

          {/* Password Field */}
          <div className={styles.fieldGroup}>
            <label htmlFor="signin-password" className={styles.label}>
              Password
            </label>
            <input
              id="signin-password"
              type="password"
              className={styles.input}
              placeholder="••••••••"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              disabled={loading}
              required
              autoComplete="current-password"
            />
          </div>

          {/* Forgot Password Link */}
          {onSwitchToReset && (
            <div className={styles.forgotPassword}>
              <button
                type="button"
                className={styles.linkButton}
                onClick={onSwitchToReset}
                disabled={loading}
              >
                Forgot password?
              </button>
            </div>
          )}

          {/* Submit Button */}
          <button
            type="submit"
            className={`glass-button ${styles.submitButton}`}
            disabled={loading || !email || !password}
          >
            {loading ? 'Signing in...' : 'Sign In'}
          </button>
        </form>

        {/* Footer */}
        <div className={styles.modalFooter}>
          <span className={styles.footerText}>Don't have an account?</span>
          <button
            className={styles.linkButton}
            onClick={onSwitchToSignUp}
            disabled={loading}
          >
            Sign up
          </button>
        </div>
      </div>
    </div>
  );
}

export default SignInModal;
