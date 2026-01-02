/**
 * OpenRouter Configuration
 * Model settings and options for translation
 */

// Available models for translation (ordered by preference)
export const TRANSLATION_MODELS = {
  // Free models (no API cost)
  free: [
    'google/gemma-3-27b-it:free',
    'meta-llama/llama-3-8b-instruct:free',
    'mistralai/mistral-7b-instruct:free',
  ],

  // Paid models (better quality)
  paid: [
    'anthropic/claude-3-haiku',
    'openai/gpt-4o-mini',
    'google/gemini-flash-1.5',
  ],

  // Premium models (best quality)
  premium: [
    'anthropic/claude-3.5-sonnet',
    'openai/gpt-4o',
    'google/gemini-pro-1.5',
  ],
};

// Default model based on tier
export const DEFAULT_MODEL = 'google/gemma-3-27b-it:free';

// Model-specific settings
export const MODEL_SETTINGS = {
  'google/gemma-3-27b-it:free': {
    temperature: 0.3,
    maxTokens: 4000,
    timeout: 30000,
  },
  'meta-llama/llama-3-8b-instruct:free': {
    temperature: 0.2,
    maxTokens: 4000,
    timeout: 25000,
  },
  'anthropic/claude-3-haiku': {
    temperature: 0.3,
    maxTokens: 8000,
    timeout: 20000,
  },
  'openai/gpt-4o-mini': {
    temperature: 0.3,
    maxTokens: 8000,
    timeout: 15000,
  },
};

// Language mappings
export const LANGUAGE_CODES = {
  ur: { name: 'Urdu', direction: 'rtl' },
  ar: { name: 'Arabic', direction: 'rtl' },
  hi: { name: 'Hindi', direction: 'ltr' },
  bn: { name: 'Bengali', direction: 'ltr' },
  fa: { name: 'Persian', direction: 'rtl' },
  es: { name: 'Spanish', direction: 'ltr' },
  fr: { name: 'French', direction: 'ltr' },
  de: { name: 'German', direction: 'ltr' },
  zh: { name: 'Chinese', direction: 'ltr' },
  ja: { name: 'Japanese', direction: 'ltr' },
};

// Get model settings with defaults
export function getModelSettings(model = DEFAULT_MODEL) {
  return {
    ...MODEL_SETTINGS[DEFAULT_MODEL],
    ...MODEL_SETTINGS[model],
    temperature: MODEL_SETTINGS[model]?.temperature || 0.3,
    maxTokens: MODEL_SETTINGS[model]?.maxTokens || 4000,
    timeout: MODEL_SETTINGS[model]?.timeout || 30000,
  };
}

// Get language info
export function getLanguageInfo(code) {
  return LANGUAGE_CODES[code] || { name: 'Unknown', direction: 'ltr' };
}

// Validate model name
export function isValidModel(model) {
  const allModels = [
    ...TRANSLATION_MODELS.free,
    ...TRANSLATION_MODELS.paid,
    ...TRANSLATION_MODELS.premium,
  ];
  return allModels.includes(model);
}
