/**
 * Profile Settings Modal Component
 * @fileoverview Modal for updating user profile information
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
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
 * Props for ProfileSettings
 */
export interface ProfileSettingsProps {
  /** Callback when modal should close */
  onClose: () => void;
}

/**
 * Profile Settings Modal Component
 */
export function ProfileSettings({
  onClose,
}: ProfileSettingsProps): JSX.Element {
  const { user, token, updateUser } = useAuth();

  const [name, setName] = useState(user?.full_name || '');
  const [email, setEmail] = useState(user?.email || '');
  const [passwordConfirm, setPasswordConfirm] = useState('');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [loading, setLoading] = useState(false);

  /**
   * Clear error when input changes
   */
  useEffect(() => {
    if (name || email || passwordConfirm) {
      setError('');
      setSuccess('');
    }
  }, [name, email, passwordConfirm]);

  /**
   * Handle profile update
   */
  const handleSubmit = async (e: React.FormEvent): Promise<void> => {
    e.preventDefault();
    setError('');
    setSuccess('');
    setLoading(true);

    try {
      // Check if email is changing
      const emailChanging = email !== user?.email;

      // If email is changing, require password confirmation
      if (emailChanging && !passwordConfirm) {
        setError('Password confirmation required for email change');
        setLoading(false);
        return;
      }

      if (emailChanging) {
        // Verify password via backend
        const verifyResponse = await fetch(`${getApiUrl()}/api/auth/signin`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            email: user?.email,
            password: passwordConfirm,
          }),
        });

        if (!verifyResponse.ok) {
          setError('Invalid password confirmation');
          setLoading(false);
          return;
        }
      }

      const response = await fetch(`${getApiUrl()}/api/auth/profile`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          full_name: name || undefined,
          email: emailChanging ? email : undefined,
          password_confirmation: emailChanging ? passwordConfirm : undefined,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Update failed');
      }

      // Update user context
      updateUser(data);

      // Clear password confirmation
      setPasswordConfirm('');

      // Show success message
      setSuccess('Profile updated successfully');

      // Clear success message after 3 seconds
      setTimeout(() => setSuccess(''), 3000);
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Update failed';
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
          <h2 className={styles.modalTitle}>Profile Settings</h2>
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

          {/* Success Message */}
          {success && (
            <div className={styles.successMessage} role="status">
              {success}
            </div>
          )}

          {/* User ID (read-only) */}
          <div className={styles.fieldGroup}>
            <label className={styles.label}>User ID</label>
            <input
              type="text"
              className={styles.input}
              value={user?.id || ''}
              disabled
              style={{ opacity: 0.6 }}
            />
          </div>

          {/* Name Field */}
          <div className={styles.fieldGroup}>
            <label htmlFor="profile-name" className={styles.label}>
              Full Name
            </label>
            <input
              id="profile-name"
              type="text"
              className={styles.input}
              placeholder="John Doe"
              value={name}
              onChange={(e) => setName(e.target.value)}
              disabled={loading}
            />
          </div>

          {/* Email Field */}
          <div className={styles.fieldGroup}>
            <label htmlFor="profile-email" className={styles.label}>
              Email
            </label>
            <input
              id="profile-email"
              type="email"
              className={styles.input}
              placeholder="you@example.com"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              disabled={loading}
            />
            {email !== user?.email && (
              <p className={styles.requirementsTitle} style={{ marginTop: '0.5rem' }}>
                ⚠️ Changing email requires password confirmation
              </p>
            )}
          </div>

          {/* Password Confirmation (shown when email changes) */}
          {email !== user?.email && (
            <div className={styles.fieldGroup}>
              <label htmlFor="profile-password" className={styles.label}>
                Confirm Password
              </label>
              <input
                id="profile-password"
                type="password"
                className={styles.input}
                placeholder="••••••••"
                value={passwordConfirm}
                onChange={(e) => setPasswordConfirm(e.target.value)}
                disabled={loading}
                required
                autoComplete="current-password"
              />
              <p className={styles.requirementsTitle} style={{ marginTop: '0.5rem', fontSize: '0.75rem' }}>
                Enter your current password to confirm email change
              </p>
            </div>
          )}

          {/* Submit Button */}
          <button
            type="submit"
            className={`glass-button ${styles.submitButton}`}
            disabled={loading || (email !== user?.email && !passwordConfirm)}
          >
            {loading ? 'Updating...' : 'Update Profile'}
          </button>
        </form>

        {/* Footer */}
        <div className={styles.modalFooter} style={{ justifyContent: 'space-between' }}>
          <button
            className={styles.linkButton}
            onClick={onClose}
            disabled={loading}
          >
            Cancel
          </button>
        </div>
      </div>
    </div>
  );
}

export default ProfileSettings;
