/**
 * Sign Up Modal Component
 * @fileoverview Modal for user registration with name, email, and password
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import styles from './AuthModal.module.css';

/**
 * Password strength levels
 */
type PasswordStrength = 'weak' | 'fair' | 'good' | 'strong';

/**
 * Props for SignUpModal
 */
export interface SignUpModalProps {
  /** Callback when modal should close */
  onClose: () => void;
  /** Callback to switch to sign in modal */
  onSwitchToSignIn: () => void;
}

/**
 * Check password strength
 */
function checkPasswordStrength(password: string): PasswordStrength {
  let score = 0;

  if (password.length >= 8) score++;
  if (password.length >= 12) score++;
  if (/[a-z]/.test(password)) score++;
  if (/[A-Z]/.test(password)) score++;
  if (/[0-9]/.test(password)) score++;
  if (/[^a-zA-Z0-9]/.test(password)) score++;

  if (score <= 2) return 'weak';
  if (score <= 3) return 'fair';
  if (score <= 4) return 'good';
  return 'strong';
}

/**
 * Sign Up Modal Component
 */
export function SignUpModal({
  onClose,
  onSwitchToSignIn,
}: SignUpModalProps): JSX.Element {
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const { signUp, clearError } = useAuth();

  const passwordStrength = password ? checkPasswordStrength(password) : null;

  // Clear error when input changes
  useEffect(() => {
    if (name || email || password || confirmPassword) {
      setError('');
    }
  }, [name, email, password, confirmPassword]);

  /**
   * Validate form
   */
  const validateForm = (): boolean => {
    if (!name.trim()) {
      setError('Please enter your name');
      return false;
    }

    if (!email.trim()) {
      setError('Please enter your email');
      return false;
    }

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
      await signUp(email, password, name);
      clearError();
      onClose();
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Registration failed';
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
          <h2 className={styles.modalTitle}>Create Account</h2>
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

          {/* Name Field */}
          <div className={styles.fieldGroup}>
            <label htmlFor="signup-name" className={styles.label}>
              Full Name
            </label>
            <input
              id="signup-name"
              type="text"
              className={styles.input}
              placeholder="John Doe"
              value={name}
              onChange={(e) => setName(e.target.value)}
              disabled={loading}
              required
              autoComplete="name"
            />
          </div>

          {/* Email Field */}
          <div className={styles.fieldGroup}>
            <label htmlFor="signup-email" className={styles.label}>
              Email
            </label>
            <input
              id="signup-email"
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
            <label htmlFor="signup-password" className={styles.label}>
              Password
            </label>
            <input
              id="signup-password"
              type="password"
              className={styles.input}
              placeholder="••••••••"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              disabled={loading}
              required
              autoComplete="new-password"
            />
            {/* Password Strength Indicator */}
            {password && (
              <div className={styles.passwordStrength}>
                <div className={styles.strengthBar}>
                  <div
                    className={`${styles.strengthFill} ${styles[passwordStrength || '']}`}
                    style={{
                      width: passwordStrength === 'weak' ? '25%' :
                             passwordStrength === 'fair' ? '50%' :
                             passwordStrength === 'good' ? '75%' : '100%'
                    }}
                  />
                </div>
                <span className={styles.strengthText}>
                  {passwordStrength?.charAt(0).toUpperCase() + passwordStrength?.slice(1)}
                </span>
              </div>
            )}
          </div>

          {/* Confirm Password Field */}
          <div className={styles.fieldGroup}>
            <label htmlFor="signup-confirm" className={styles.label}>
              Confirm Password
            </label>
            <input
              id="signup-confirm"
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

          {/* Password Requirements */}
          <div className={styles.requirements}>
            <span className={styles.requirementsTitle}>Password must contain:</span>
            <ul className={styles.requirementsList}>
              <li className={password.length >= 8 ? styles.met : ''}>
                At least 8 characters
              </li>
              <li className={/[A-Z]/.test(password) ? styles.met : ''}>
                Uppercase letter
              </li>
              <li className={/[a-z]/.test(password) ? styles.met : ''}>
                Lowercase letter
              </li>
              <li className={/[0-9]/.test(password) ? styles.met : ''}>
                Number
              </li>
            </ul>
          </div>

          {/* Submit Button */}
          <button
            type="submit"
            className={`glass-button ${styles.submitButton}`}
            disabled={loading || !name || !email || !password || !confirmPassword}
          >
            {loading ? 'Creating account...' : 'Create Account'}
          </button>
        </form>

        {/* Footer */}
        <div className={styles.modalFooter}>
          <span className={styles.footerText}>Already have an account?</span>
          <button
            className={styles.linkButton}
            onClick={onSwitchToSignIn}
            disabled={loading}
          >
            Sign in
          </button>
        </div>
      </div>
    </div>
  );
}

export default SignUpModal;
