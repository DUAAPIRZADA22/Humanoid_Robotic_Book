/**
 * Text processing utilities for chat widget
 * @fileoverview Functions for processing and sanitizing text
 */

/**
 * Strip HTML tags from text
 * @param html - HTML string to clean
 * @returns Plain text
 */
export function stripHtml(html: string): string {
  const temp = document.createElement('div');
  temp.innerHTML = html;
  return temp.textContent || temp.innerText || '';
}

/**
 * Sanitize text for API request
 * @param text - Text to sanitize
 * @returns Sanitized text
 */
export function sanitizeText(text: string): string {
  return text
    // Remove control characters
    .replace(/[\x00-\x1F\x7F]/g, '')
    // Normalize whitespace
    .replace(/\s+/g, ' ')
    // Trim
    .trim();
}

/**
 * Check if text contains only whitespace
 * @param text - Text to check
 * @returns True if text is empty or only whitespace
 */
export function isEmpty(text: string): boolean {
  return !text || /^\s*$/.test(text);
}

/**
 * Truncate text with ellipsis
 * @param text - Text to truncate
 * @param maxLength - Maximum length
 * @param suffix - Suffix to add (default: '...')
 * @returns Truncated text
 */
export function truncateText(
  text: string,
  maxLength: number,
  suffix: string = '...'
): string {
  if (text.length <= maxLength) {
    return text;
  }

  const truncateLength = Math.max(0, maxLength - suffix.length);
  return `${text.substring(0, truncateLength)}${suffix}`;
}

/**
 * Extract meaningful text from selection
 * @param selection - Window Selection object
 * @returns Processed text
 */
export function extractSelectedText(selection: Selection): string {
  if (!selection || selection.isCollapsed) {
    return '';
  }

  let text = selection.toString();

  // Sanitize text
  text = sanitizeText(text);

  // Validate minimum length
  if (text.length < 3) {
    return '';
  }

  return text;
}

/**
 * Check if selection is meaningful (not too short, not just whitespace)
 * @param selection - Window Selection object
 * @returns True if selection is meaningful
 */
export function isMeaningfulSelection(selection: Selection): boolean {
  if (!selection || selection.isCollapsed) {
    return false;
  }

  const text = extractSelectedText(selection);

  // Check minimum length and meaningful content
  return text.length >= 3 && !/^\s+$/.test(text);
}

/**
 * Get context around selected text
 * @param text - Full text content
 * @param selectedText - Text that was selected
 * @param contextLength - Number of characters before and after
 * @returns Context object
 */
export function getTextContext(
  text: string,
  selectedText: string,
  contextLength: number = 100
): {
  before: string;
  after: string;
  full: string;
} {
  const index = text.indexOf(selectedText);

  if (index === -1) {
    return {
      before: '',
      after: '',
      full: selectedText,
    };
  }

  const start = Math.max(0, index - contextLength);
  const end = Math.min(text.length, index + selectedText.length + contextLength);

  const before = text.substring(start, index);
  const after = text.substring(index + selectedText.length, end);

  return {
    before: before.trim(),
    after: after.trim(),
    full: text.substring(start, end),
  };
}

/**
 * Detect if text is code
 * @param text - Text to check
 * @returns True if text appears to be code
 */
export function isLikelyCode(text: string): boolean {
  const codeIndicators = [
    /\bfunction\s+\w+\s*\(/,
    /\bclass\s+\w+/,
    /\bdef\s+\w+\s*\(/,
    /\bimport\s+\w+/,
    /\bexport\s+/,
    /\bconst\s+\w+\s*=/,
    /\blet\s+\w+\s*=/,
    /\bvar\s+\w+\s*=/,
    /\{[\s\S]*\}/, // Object literal
    /\[[\s\S]*\]/, // Array literal
    /\/\/.*$/, // Single line comment
    /\/\*[\s\S]*\*\//, // Multi-line comment
    /<[^>]+>/, // HTML-like tags
  ];

  // Check if text matches any code patterns
  const hasCodePatterns = codeIndicators.some(pattern => pattern.test(text));

  // Check character distribution (code often has more special characters)
  const specialCharCount = (text.match(/[^a-zA-Z\s]/g) || []).length;
  const specialCharRatio = specialCharCount / text.length;

  return hasCodePatterns || specialCharRatio > 0.2;
}

/**
 * Clean and normalize text for display
 * @param text - Text to clean
 * @returns Cleaned text
 */
export function cleanDisplayText(text: string): string {
  return text
    // Normalize line endings
    .replace(/\r\n/g, '\n')
    .replace(/\r/g, '\n')
    // Remove excessive whitespace
    .replace(/[ \t]+/g, ' ')
    // Remove excessive newlines
    .replace(/\n{3,}/g, '\n\n')
    // Trim
    .trim();
}

/**
 * Format text for AI context
 * @param selectedText - Selected text
 * @param context - Optional context information
 * @returns Formatted context string
 */
export function formatContextForAI(
  selectedText: string,
  context?: {
    pageUrl?: string;
    pageTitle?: string;
    before?: string;
    after?: string;
  }
): string {
  let formattedContext = '';

  if (context?.pageTitle) {
    formattedContext += `Page: ${context.pageTitle}\n`;
  }

  if (context?.pageUrl) {
    formattedContext += `URL: ${context.pageUrl}\n`;
  }

  formattedContext += '\nSelected text:\n';
  formattedContext += `"${selectedText}"`;

  if (context?.before || context?.after) {
    formattedContext += '\n\nContext:';

    if (context?.before) {
      formattedContext += `\nBefore: ...${context.before}`;
    }

    if (context?.after) {
      formattedContext += `\nAfter: ${context.after}...`;
    }
  }

  return formattedContext.trim();
}

/**
 * Extract keywords from text
 * @param text - Text to analyze
 * @param maxKeywords - Maximum number of keywords to extract
 * @returns Array of keywords
 */
export function extractKeywords(text: string, maxKeywords: number = 5): string[] {
  // Simple keyword extraction based on word frequency and importance
  const words = text
    .toLowerCase()
    .replace(/[^\w\s]/g, ' ')
    .split(/\s+/)
    .filter(word => word.length > 3);

  // Common stop words to filter out
  const stopWords = new Set([
    'the', 'and', 'for', 'are', 'but', 'not', 'you', 'all', 'can', 'had',
    'her', 'was', 'one', 'our', 'out', 'day', 'get', 'has', 'him', 'his',
    'how', 'man', 'new', 'now', 'old', 'see', 'two', 'way', 'who', 'boy',
    'did', 'its', 'let', 'put', 'say', 'she', 'too', 'use', 'her', 'many',
    'than', 'them', 'well', 'were', 'will', 'with', 'have', 'this', 'that',
    'from', 'they', 'know', 'want', 'been', 'good', 'much', 'some', 'time',
    'very', 'when', 'come', 'here', 'just', 'like', 'long', 'make', 'many',
    'more', 'most', 'only', 'other', 'over', 'such', 'take', 'than', 'their',
    'them', 'then', 'there', 'these', 'they', 'think', 'which', 'while', 'about',
    'after', 'again', 'against', 'because', 'before', 'being', 'below', 'between',
    'both', 'cannot', 'could', 'doing', 'during', 'each', 'few', 'further',
    'none', 'same', 'since', 'some', 'still', 'those', 'through', 'under', 'until',
    'where', 'while', 'would',
  ]);

  // Count word frequencies
  const wordFreq = new Map<string, number>();
  words.forEach(word => {
    if (!stopWords.has(word)) {
      wordFreq.set(word, (wordFreq.get(word) || 0) + 1);
    }
  });

  // Sort by frequency and return top keywords
  return Array.from(wordFreq.entries())
    .sort((a, b) => b[1] - a[1])
    .slice(0, maxKeywords)
    .map(([word]) => word);
}