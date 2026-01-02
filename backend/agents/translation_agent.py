"""
AI Translation Service using OpenRouter.

This module provides intelligent translation capabilities that:
- Preserve Markdown structure
- Skip code blocks and inline code
- Handle technical content appropriately
"""

import re
import os
from typing import Optional
from openai import OpenAI


# ============================================================================
# TEXT PROCESSING TOOLS
# ============================================================================

def extract_segments(text: str) -> list[dict]:
    """
    Extract text segments, preserving code blocks and inline code.

    Returns a list of segments:
    - {"type": "code", "content": "..."} - code blocks (skipped in translation)
    - {"type": "inline_code", "content": "..."} - inline code (skipped)
    - {"type": "text", "content": "..."} - translatable text
    """
    segments = []
    pattern = r'(```[\s\S]*?```|`[^`]+`)'

    parts = re.split(pattern, text)

    for part in parts:
        if not part:
            continue

        if part.startswith('```') and part.endswith('```'):
            segments.append({"type": "code", "content": part})
        elif part.startswith('`') and part.endswith('`'):
            segments.append({"type": "inline_code", "content": part})
        else:
            segments.append({"type": "text", "content": part})

    return segments


def reconstruct_text(segments: list[dict], translated_segments: list[str]) -> str:
    """Reconstruct the full text with translated and original segments."""
    result = []
    translatable_idx = 0

    for segment in segments:
        if segment["type"] == "text":
            if translatable_idx < len(translated_segments):
                result.append(translated_segments[translatable_idx])
                translatable_idx += 1
            else:
                result.append(segment["content"])
        else:
            result.append(segment["content"])

    return "".join(result)


# ============================================================================
# TRANSLATION SERVICE (using OpenRouter)
# ============================================================================

class TranslationService:
    """Service for handling translation requests via OpenRouter API."""

    def __init__(self):
        """Initialize the translation service."""
        self.client = None
        self.api_key = os.getenv("OPENROUTER_API_KEY")
        self.base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")
        self.model = os.getenv("TRANSLATION_MODEL", "google/gemma-3-27b-it:free")
        self._init_client()

    def _init_client(self):
        """Initialize OpenAI client with OpenRouter configuration."""
        if self.api_key:
            try:
                self.client = OpenAI(
                    api_key=self.api_key,
                    base_url=self.base_url,
                )
                print(f"[TranslationService] Initialized with model: {self.model}")
            except Exception as e:
                print(f"[TranslationService] Failed to initialize client: {e}")

    def _get_system_prompt(self, target_lang: str) -> str:
        """Get system prompt for translation."""
        lang_names = {
            "ur": "Urdu",
            "es": "Spanish",
            "fr": "French",
            "ar": "Arabic",
            "hi": "Hindi",
            "bn": "Bengali",
            "fa": "Persian",
            "zh": "Chinese",
            "ja": "Japanese",
        }

        target_name = lang_names.get(target_lang, "Urdu")

        return f"""You are a professional technical document translator for a book on Physical AI and Humanoid Robotics.

# TRANSLATION RULES

1. **Preserve Markdown Structure**: Keep all headings (#, ##, ###), bold (**), italic (*), lists (-, 1.), and links ([text](url))

2. **DO NOT Translate Code Blocks**: Keep code blocks unchanged

3. **DO NOT Translate Inline Code**: Keep inline code unchanged

4. **Preserve Technical Terms**: Keep terms like: ROS, OpenCV, PyBullet, Isaac Sim, NumPy, TensorFlow, PyTorch

5. **Preserve Commands**: Keep commands like npm install, pip install, python, etc.

6. **Translation Quality**: Use natural, flowing {target_name}. For technical terms without common translations, keep the English term.

7. **Output Format**: Return ONLY the translated text. NO explanations, NO notes, NO conversational filler.

Translate the given text to {target_name}."""

    def _translate_chunk(self, text: str, target_lang: str = "Urdu") -> str:
        """Translate a single chunk of text."""
        if not self.client:
            raise ValueError("OpenRouter client not initialized - missing API key")

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {
                    "role": "system",
                    "content": self._get_system_prompt(target_lang)
                },
                {
                    "role": "user",
                    "content": text
                }
            ],
            temperature=0.3,
            max_tokens=8000
        )

        return response.choices[0].message.content.strip()

    async def translate_text(self, text: str, target_lang: str = "ur") -> dict:
        """
        Translate text content using OpenRouter API.

        Args:
            text: Markdown content to translate
            target_lang: Target language code (ur, es, fr, ar, etc.)

        Returns:
            Dict with:
                - success: bool
                - translated_text: str
                - original_text: str
                - language: str
        """
        if not text or not text.strip():
            return {
                "success": False,
                "error": "Empty text provided"
            }

        if not self.client:
            return {
                "success": False,
                "error": "Translation service not available - OPENROUTER_API_KEY missing"
            }

        try:
            # Extract segments (separate code from text)
            segments = extract_segments(text)

            # Collect translatable segments
            translatable_texts = [
                s["content"] for s in segments if s["type"] == "text" and s["content"].strip()
            ]

            if not translatable_texts:
                # Nothing to translate (all code)
                return {
                    "success": True,
                    "translated_text": text,
                    "original_text": text,
                    "language": target_lang,
                    "message": "No translatable content found"
                }

            # Translate each translatable chunk
            translated_chunks = []
            for i, chunk in enumerate(translatable_texts):
                if len(chunk.strip()) > 0:
                    print(f"[TranslationService] Translating chunk {i+1}/{len(translatable_texts)}...")
                    translated = self._translate_chunk(chunk, target_lang)
                    translated_chunks.append(translated)
                else:
                    translated_chunks.append("")

            # Reconstruct full translated text
            translated_text = reconstruct_text(segments, translated_chunks)

            return {
                "success": True,
                "translated_text": translated_text,
                "original_text": text,
                "language": target_lang
            }

        except Exception as e:
            print(f"[TranslationService] Error: {e}")
            return {
                "success": False,
                "error": str(e)
            }


# Singleton instance
_translation_service: Optional[TranslationService] = None


def get_translation_service() -> TranslationService:
    """Get the singleton translation service."""
    global _translation_service
    if _translation_service is None:
        _translation_service = TranslationService()
    return _translation_service


__all__ = [
    "TranslationService",
    "get_translation_service"
]
