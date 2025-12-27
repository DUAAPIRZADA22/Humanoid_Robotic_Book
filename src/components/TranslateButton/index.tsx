/**
 * Translate Button Component (Agent-based)
 * @fileoverview Translates pages using OpenAI Agents SDK backend
 */

import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

interface TranslateButtonProps {
  className?: string;
  targetLang?: string;
  languageName?: string;
}

/**
 * Gets the raw markdown content of the current page
 */
function getPageMarkdown(): string {
  // Try to get markdown from Docusaurus internal data
  const docsMarkdown = (window as any).__DOCUSAURUS Docs?.markdown;

  if (docsMarkdown) {
    return docsMarkdown;
  }

  // Fallback: Get content from article and convert to markdown-ish format
  const article = document.querySelector('article');
  if (!article) return '';

  // Extract structured content
  const headings = article.querySelectorAll('h1, h2, h3, h4');
  const paragraphs = article.querySelectorAll('p');
  const codeBlocks = article.querySelectorAll('pre');
  const lists = article.querySelectorAll('ul, ol');

  let markdown = '';

  headings.forEach((h: HTMLElement) => {
    const level = h.tagName.toLowerCase();
    const text = h.textContent || '';
    markdown += `${level.repeat(parseInt(level[1]))} ${text}\n\n`;
  });

  codeBlocks.forEach((pre: HTMLElement) => {
    const code = pre.querySelector('code');
    if (code) {
      const lang = code.className.replace('language-', '') || '';
      const text = code.textContent || '';
      markdown += `\`\`\`${lang}\n${text}\n\`\`\`\n\n`;
    }
  });

  paragraphs.forEach((p: HTMLElement) => {
    const text = p.textContent || '';
    if (text.trim()) {
      markdown += `${text}\n\n`;
    }
  });

  return markdown;
}

/**
 * Replaces page content with translated markdown
 * NOTE: This is a simplified approach - for production, you'd want
 * to use a proper markdown renderer like react-markdown
 */
function replacePageContent(markdown: string) {
  const article = document.querySelector('article') as HTMLElement;
  if (!article) return;

  // Simple markdown to HTML conversion
  let html = markdown
    // Code blocks
    .replace(/```(\w*)\n([\s\S]*?)```/g, '<pre><code class="language-$1">$2</code></pre>')
    // Inline code
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    // Bold
    .replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>')
    // Italic
    .replace(/\*([^*]+)\*/g, '<em>$1</em>')
    // Headings
    .replace(/^### (.+)$/gm, '<h3>$1</h3>')
    .replace(/^## (.+)$/gm, '<h2>$1</h2>')
    .replace(/^# (.+)$/gm, '<h1>$1</h1>')
    // Paragraphs (double newline)
    .replace(/\n\n/g, '</p><p>')
    // Line breaks
    .replace(/\n/g, '<br>');

  // Wrap in p tag
  if (!html.startsWith('<')) {
    html = '<p>' + html + '</p>';
  }

  article.innerHTML = html;

  // Set RTL for Urdu/Arabic
  const isRTL = ['ur', 'ar', 'fa'].includes('ur');
  article.setAttribute('dir', isRTL ? 'rtl' : 'ltr');
  document.documentElement.setAttribute('dir', isRTL ? 'rtl' : 'ltr');
}

export default function TranslateButton({
  className = '',
  targetLang = 'ur',
  languageName = 'Urdu',
}: TranslateButtonProps): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [originalMarkdown, setOriginalMarkdown] = useState<string>('');
  const [isClient, setIsClient] = useState(false);

  const translationControllerRef = useRef<AbortController | null>(null);

  useEffect(() => {
    setIsClient(true);
  }, []);

  /**
   * Handle translation using Agent-based backend
   */
  const handleTranslate = async () => {
    if (isTranslating || typeof window === 'undefined') return;

    // Get page markdown content
    const markdown = getPageMarkdown();
    if (!markdown || markdown.trim().length < 10) {
      setError('No content to translate');
      setTimeout(() => setError(null), 3000);
      return;
    }

    setIsTranslating(true);
    setError(null);

    try {
      // Create abort controller for timeout
      const controller = new AbortController();
      translationControllerRef.current = controller;

      const timeoutId = setTimeout(() => controller.abort(), 60000); // 60s timeout

      // Get page slug from URL
      const pageSlug = window.location.pathname.replace(/\/$/, '') || 'index';

      // Get API URL from config
      const baseUrl = (siteConfig?.customFields?.translateApiUrl as string) ||
                     'http://localhost:8000';

      console.log('[TranslateButton] Sending translation request:', { pageSlug, targetLang, contentLength: markdown.length });

      // Call translation endpoint (backend /api/translate)
      const response = await fetch(`${baseUrl}/api/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          content: markdown,
          pageSlug,
          targetLang,
        }),
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.error || `HTTP ${response.status}`);
      }

      const data = await response.json();

      if (data.success && data.translated_text) {
        // Store original markdown for restore
        if (!isTranslated) {
          setOriginalMarkdown(markdown);
        }

        // Replace page content with translated markdown
        replacePageContent(data.translated_text);

        setIsTranslated(true);
        console.log('[TranslateButton] Translation complete');
      } else {
        throw new Error(data.error || 'Translation failed');
      }

    } catch (err) {
      const message = err instanceof Error ? err.message : 'Unknown error';
      console.error('[TranslateButton] Error:', err);

      if (message.includes('abort') || message.includes('timeout')) {
        setError('Translation timed out');
      } else if (message.includes('fetch')) {
        setError('Translation service unavailable - ensure backend is running');
      } else {
        setError(message);
      }

      setTimeout(() => setError(null), 5000);
    } finally {
      setIsTranslating(false);
      translationControllerRef.current = null;
    }
  };

  /**
   * Restore original content
   */
  const handleRestore = async () => {
    if (originalMarkdown) {
      replacePageContent(originalMarkdown);
      setIsTranslated(false);
      setError(null);
    }
  };

  /**
   * Reset on navigation
   */
  useEffect(() => {
    const handleRouteChange = () => {
      setIsTranslated(false);
      setOriginalMarkdown('');
      setError(null);
      if (typeof window !== 'undefined') {
        document.documentElement.setAttribute('dir', 'ltr');
      }
    };

    window.addEventListener('routeChangeComplete', handleRouteChange);

    return () => {
      window.removeEventListener('routeChangeComplete', handleRouteChange);
      // Abort any ongoing translation
      if (translationControllerRef.current) {
        translationControllerRef.current.abort();
      }
    };
  }, []);

  // Server-side fallback
  if (!isClient) {
    return (
      <button
        className={`clean-btn ${styles.translateButton} ${className}`}
        disabled
        aria-label="Translate"
      >
        🌐
      </button>
    );
  }

  return (
    <>
      <button
        className={`clean-btn ${styles.translateButton} ${className} ${isTranslated ? styles.translated : ''}`}
        onClick={isTranslated ? handleRestore : handleTranslate}
        disabled={isTranslating}
        title={isTranslated ? 'Restore original' : `Translate to ${languageName}`}
      >
        {isTranslating ? (
          <>
            <span className={styles.spinner} aria-hidden="true" />
            Translating...
          </>
        ) : isTranslated ? (
          <>
            <span className={styles.icon}>↩</span>
            English
          </>
        ) : (
          <>
            <span className={styles.icon}>🌐</span>
            {languageName}
          </>
        )}
      </button>

      {error && (
        <div
          className={styles.errorToast}
          onClick={() => setError(null)}
          role="alert"
        >
          <span className={styles.errorIcon}>⚠️</span>
          <span>{error}</span>
          <span className={styles.closeIcon}>✕</span>
        </div>
      )}
    </>
  );
}

/**
 * Hook to use translation programmatically
 */
export function usePageTranslation() {
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState<string>('');

  const translate = async (markdown: string, pageSlug: string, targetLang = 'ur') => {
    const apiUrl = import.meta.env.TRANSLATE_API_URL || 'http://localhost:3001/api';

    const response = await fetch(`${apiUrl}/translation/translate`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ content: markdown, pageSlug, targetLang }),
    });

    const data = await response.json();
    if (data.success) {
      setIsTranslated(true);
      setTranslatedContent(data.translatedContent);
      return data.translatedContent;
    }
    throw new Error(data.error || 'Translation failed');
  };

  const restore = () => {
    setIsTranslated(false);
    setTranslatedContent('');
  };

  return { isTranslated, translatedContent, translate, restore };
}
