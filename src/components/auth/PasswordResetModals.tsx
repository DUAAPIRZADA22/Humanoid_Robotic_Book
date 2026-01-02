/**
 * Password Reset Modals Component
 * @fileoverview Modals for requesting and confirming password reset
 */

import React, { useState, useEffect } from 'react';
import styles from './AuthModal.module.css';

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
 * Props for ResetRequestModal
 */
export interface ResetRequestModalProps {
  /** Callback when modal should close */
  onClose: () => void;
  /** Callback when reset is requested */
  onResetRequested?: (email: string) => void;
}

/**
 * Password Reset Request Modal
 */
export function ResetRequestModal({
  onClose,
  onResetRequested,
}: ResetRequestModalProps): JSX.Element {
  const [email, setEmail] = useState('');
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const [success, setSuccess] = useState(false);

  /**
   * Handle form submission
   */
  const handleSubmit = async (e: React.FormEvent): Promise<void> => {
    e.preventDefault();
    setError('');
    setMessage('');
    setLoading(true);

    try {
      const response = await fetch(`${getApiUrl()}/api/auth/reset-request`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Request failed');
      }

      setSuccess(true);
      setMessage(data.message);

      if (onResetRequested) {
        onResetRequested(email);
      }

      // For development/testing: show reset URL
      if (data.reset_url) {
        console.log('Reset URL:', data.reset_url);
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Request failed';
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
          <h2 className={styles.modalTitle}>Reset Password</h2>
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
        {!success ? (
          <form onSubmit={handleSubmit} className={styles.form}>
            <p className={styles.footerText} style={{ textAlign: 'center' }}>
              Enter your email address and we'll send you a link to reset your password.
            </p>

            {/* Error Message */}
            {error && (
              <div className={styles.errorMessage} role="alert">
                {error}
              </div>
            )}

            {/* Email Field */}
            <div className={styles.fieldGroup}>
              <label htmlFor="reset-email" className={styles.label}>
                Email
              </label>
              <input
                id="reset-email"
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

            {/* Submit Button */}
            <button
              type="submit"
              className={`glass-button ${styles.submitButton}`}
              disabled={loading || !email}
            >
              {loading ? 'Sending...' : 'Send Reset Link'}
            </button>
          </form>
        ) : (
          <div className={styles.form}>
            <div className={styles.successMessage} role="status">
              {message}
            </div>
            <button
              className={`glass-button ${styles.submitButton}`}
              onClick={onClose}
            >
              Close
            </button>
          </div>
        )}
      </div>
    </div>
  );
}

/**
 * Props for ResetPasswordModal
 */
export interface ResetPasswordModalProps {
  /** Password reset token from URL */
  token: string;
  /** Callback when modal should close */
  onClose: () => void;
  /** Callback when password is successfully reset */
  onResetSuccess?: () => void;
}

/**
 * Password Reset Confirmation Modal
 */
export function ResetPasswordModal({
  token,
  onClose,
  onResetSuccess,
}: ResetPasswordModalProps): JSX.Element {
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const [success, setSuccess] = useState(false);

  /**
   * Validate form
   */
  const validateForm = (): boolean => {
    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return false;
    }

    if (!/[A-Z]/.test(password)) {
      setError('Password must contain at least one uppercase letter');
      return false;
    }

    if (!/[a-z]/.test(password)) {
      setError('Password must contain at least one lowercase letter');
      return false;
    }

    if (!/[0-9]/.test(password)) {
      setError('Password must contain at least one number');
      return false;
    }

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return false;
    }

    return true;
  };

  /**
   * Handle form submission
   */
  const handleSubmit = async (e: React.FormEvent): Promise<void> => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setError('');
    setLoading(true);

    try {
      const response = await fetch(`${getApiUrl()}/api/auth/reset-confirm`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          token,
          new_password: password,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Reset failed');
      }

      setSuccess(true);

      if (onResetSuccess) {
        setTimeout(() => onResetSuccess(), 1500);
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Reset failed';
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
          <h2 className={styles.modalTitle}>Set New Password</h2>
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
        {!success ? (
          <form onSubmit={handleSubmit} className={styles.form}>
            {/* Error Message */}
            {error && (
              <div className={styles.errorMessage} role="alert">
                {error}
              </div>
            )}

            {/* Password Field */}
            <div className={styles.fieldGroup}>
              <label htmlFor="reset-password" className={styles.label}>
                New Password
              </label>
              <input
                id="reset-password"
                type="password"
                className={styles.input}
                placeholder="••••••••"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                disabled={loading}
                required
                autoComplete="new-password"
              />
            </div>

            {/* Confirm Password Field */}
            <div className={styles.fieldGroup}>
              <label htmlFor="reset-confirm" className={styles.label}>
                Confirm New Password
              </label>
              <input
                id="reset-confirm"
                type="password"
                className={styles.input}
                placeholder="••••••••"
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                disabled={loading}
                required
                autoComplete="new-password"
              />
            </div>

            {/* Submit Button */}
            <button
              type="submit"
              className={`glass-button ${styles.submitButton}`}
              disabled={loading || !password || !confirmPassword}
            >
              {loading ? 'Resetting...' : 'Reset Password'}
            </button>
          </form>
        ) : (
          <div className={styles.form}>
            <div className={styles.successMessage} role="status">
              Password reset successfully! You can now sign in with your new password.
            </div>
            <button
              className={`glass-button ${styles.submitButton}`}
              onClick={onClose}
            >
              Close
            </button>
          </div>
        )}
      </div>
    </div>
  );
}

export default ResetPasswordModal;
