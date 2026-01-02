/**
 * Navbar wrapper with authentication buttons
 * @fileoverview Extends Docusaurus navbar to add Sign In/Sign Up buttons
 */

import React, { useEffect, useState, useRef } from 'react';
import { createPortal } from 'react-dom';
import Navbar from '@theme-original/Navbar';
import { SignInModal, SignUpModal } from '../../components/auth';
import styles from './styles.module.css';

export default function NavbarWrapper(props) {
  const [userEmail, setUserEmail] = useState<string | null>(null);
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [authMode, setAuthMode] = useState<'signin' | 'signup'>('signin');
  const [navbarEnd, setNavbarEnd] = useState<HTMLElement | null>(null);
  const authButtonsRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    // Check localStorage for user data
    const checkAuth = () => {
      try {
        const userStr = localStorage.getItem('auth_user');
        if (userStr) {
          const user = JSON.parse(userStr);
          setUserEmail(user.email);
        } else {
          setUserEmail(null);
        }
      } catch (e) {
        setUserEmail(null);
      }
    };

    checkAuth();

    // Listen for auth changes
    const handleAuthChange = () => {
      checkAuth();
    };

    const handleSignInClick = () => {
      setAuthMode('signin');
      setShowAuthModal(true);
    };

    window.addEventListener('auth:signin', handleAuthChange);
    window.addEventListener('auth:signout', handleAuthChange);
    window.addEventListener('auth:required', handleSignInClick);
    window.addEventListener('storage', checkAuth);

    return () => {
      window.removeEventListener('auth:signin', handleAuthChange);
      window.removeEventListener('auth:signout', handleAuthChange);
      window.removeEventListener('auth:required', handleSignInClick);
      window.removeEventListener('storage', checkAuth);
    };
  }, []);

  // Find and inject auth buttons into navbar
  useEffect(() => {
    // Wait for navbar to be rendered
    const findNavbarEnd = () => {
      const navbarEndEl = document.querySelector('.navbar__items--right');
      if (navbarEndEl) {
        setNavbarEnd(navbarEndEl as HTMLElement);
      }
    };

    // Try immediately and then with delays
    findNavbarEnd();
    const timeout1 = setTimeout(findNavbarEnd, 100);
    const timeout2 = setTimeout(findNavbarEnd, 500);

    // Re-find on navigation
    const observer = new MutationObserver(() => {
      findNavbarEnd();
    });

    observer.observe(document.body, { childList: true, subtree: true });

    return () => {
      clearTimeout(timeout1);
      clearTimeout(timeout2);
      observer.disconnect();
    };
  }, []);

  const handleSignInClick = () => {
    setAuthMode('signin');
    setShowAuthModal(true);
  };

  const handleSignUpClick = () => {
    setAuthMode('signup');
    setShowAuthModal(true);
  };

  const handleSignOut = () => {
    localStorage.removeItem('auth_token');
    localStorage.removeItem('auth_user');
    setUserEmail(null);
    window.dispatchEvent(new CustomEvent('auth:signout'));
    window.location.reload();
  };

  // Create auth buttons element
  const authButtons = (
    <div ref={authButtonsRef} className={styles.authButtonsContainer}>
      {userEmail ? (
        <div className={styles.userMenu}>
          <span className={styles.userEmail}>{userEmail}</span>
          <button
            className={`glass-button ${styles.signOutButton}`}
            onClick={handleSignOut}
          >
            Sign Out
          </button>
        </div>
      ) : (
        <>
          <button
            className={`glass-button ${styles.signInButton}`}
            onClick={handleSignInClick}
          >
            Sign In
          </button>
          <button
            className={`glass-button ${styles.signUpButton}`}
            onClick={handleSignUpClick}
          >
            Sign Up
          </button>
        </>
      )}
    </div>
  );

  return (
    <>
      <Navbar {...props} />

      {/* Render auth buttons via portal into navbar */}
      {navbarEnd && createPortal(authButtons, navbarEnd)}

      {/* Auth Modals */}
      {showAuthModal && (
        <>
          {authMode === 'signin' ? (
            <SignInModal
              onClose={() => setShowAuthModal(false)}
              onSignUpClick={() => setAuthMode('signup')}
            />
          ) : (
            <SignUpModal
              onClose={() => setShowAuthModal(false)}
              onSignInClick={() => setAuthMode('signin')}
            />
          )}
        </>
      )}
    </>
  );
}
