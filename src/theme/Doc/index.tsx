/**
 * Doc wrapper with Translate Button
 * @fileoverview Adds Translate Button to documentation pages only
 */

import React from 'react';
import Doc from '@theme-original/Doc';
import TranslateButton from '../../components/TranslateButton';
import { useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

export default function DocWrapper(props): JSX.Element {
  const location = useLocation();

  // Check if this is NOT homepage - show button on all content pages except home
  const isHomepage = location.pathname === '/' || location.pathname === '/index';
  const isBlog = location.pathname.startsWith('/blog');
  const isTags = location.pathname.startsWith('/tags');

  // Show translate button on doc pages and other content pages, but not homepage
  const shouldShowTranslate = !isHomepage && !isBlog && !isTags;

  // If we don't want to show the button, just render the original Doc
  if (!shouldShowTranslate) {
    return <Doc {...props} />;
  }

  return (
    <div className={styles.docPageWrapper}>
      {/* Translate Button floating on top right of content */}
      <div className={styles.translateButtonContainer}>
        <TranslateButton />
      </div>
      <Doc {...props} />
    </div>
  );
}
