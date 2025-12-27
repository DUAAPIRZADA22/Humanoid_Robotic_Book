"""
LLM service for generating answers based on retrieved context.
Supports multiple LLM providers with streaming capabilities.
"""

import os
import asyncio
import logging
from typing import AsyncGenerator, List, Dict, Any, Optional
import json

logger = logging.getLogger(__name__)


class LLMService:
    """
    Service for generating answers using Large Language Models.
    Supports OpenAI, Cohere, and OpenRouter APIs.
    """

    def __init__(
        self,
        provider: str = "openrouter",
        api_key: Optional[str] = None,
        model: str = "mistralai/devstral-2512:free",
        base_url: Optional[str] = None,
        max_tokens: int = 1000,
        temperature: float = 0.7
    ):
        """
        Initialize the LLM service.

        Args:
            provider: LLM provider (openrouter, cohere, openai, anthropic, etc.)
            api_key: API key for the provider
            model: Model name to use
            base_url: Custom base URL for OpenAI-compatible APIs
            max_tokens: Maximum tokens in response
            temperature: Response randomness (0-1)
        """
        self.provider = provider
        self.api_key = api_key or os.getenv(f"{provider.upper()}_API_KEY")
        self.model = model
        self.base_url = base_url
        self.max_tokens = max_tokens
        self.temperature = temperature

        # Initialize based on provider
        if provider == "openrouter":
            self._init_openrouter()
        elif provider == "cohere":
            self._init_cohere()
        elif provider == "openai":
            self._init_openai()
        else:
            raise ValueError(f"Unsupported provider: {provider}")

    def _init_openrouter(self):
        """Initialize OpenRouter client (uses OpenAI-compatible API)."""
        try:
            import openai
            self.client = openai.AsyncOpenAI(
                api_key=self.api_key,
                base_url=self.base_url
            )
        except ImportError:
            logger.error("OpenAI package not installed. Install with: pip install openai")
            raise

    def _init_cohere(self):
        """Initialize Cohere client."""
        try:
            import cohere
            import aiohttp
            # Use synchronous client for now with async wrapper
            self.client = cohere.ClientV2(api_key=self.api_key)
        except ImportError:
            logger.error("Cohere package not installed. Install with: pip install cohere")
            raise

    def _init_openai(self):
        """Initialize OpenAI client."""
        try:
            import openai
            self.client = openai.AsyncOpenAI(
                api_key=self.api_key,
                base_url=self.base_url
            )
        except ImportError:
            logger.error("OpenAI package not installed. Install with: pip install openai")
            raise

    async def generate_answer(
        self,
        query: str,
        context: List[Dict[str, Any]],
        stream: bool = True
    ) -> AsyncGenerator[str, None]:
        """
        Generate an answer based on the query and retrieved context.

        Args:
            query: User's question
            context: Retrieved document chunks
            stream: Whether to stream the response

        Yields:
            Response content chunks
        """
        # Build context string
        context_text = self._build_context(context)

        # Create system prompt
        system_prompt = """You are a helpful AI assistant answering questions about Physical AI and Humanoid Robotics.
Use the provided context to answer the user's question accurately and comprehensively.
If the context doesn't contain enough information, say so politely.
Always base your answers on the provided context."""

        # Create user prompt
        user_prompt = f"""Context:
{context_text}

Question: {query}

Please provide a comprehensive answer based on the context above."""

        try:
            if self.provider == "openrouter":
                async for chunk in self._generate_openrouter_stream(
                    system_prompt, user_prompt
                ):
                    yield chunk
            elif self.provider == "cohere":
                async for chunk in self._generate_cohere_stream(
                    system_prompt, user_prompt
                ):
                    yield chunk
            elif self.provider == "openai":
                async for chunk in self._generate_openai_stream(
                    system_prompt, user_prompt
                ):
                    yield chunk
        except Exception as e:
            logger.error(f"Error generating answer: {e}")
            yield f"Error generating answer: {str(e)}"

    def _build_context(self, context: List[Dict[str, Any]]) -> str:
        """Build a formatted context string from retrieved chunks."""
        if not context:
            return "No relevant context found."

        context_parts = []
        for i, chunk in enumerate(context, 1):
            text = chunk.get("text", "")
            source = chunk.get("metadata", {}).get("source", "Unknown")
            context_parts.append(f"[Source {i}: {source}]\n{text}")

        return "\n\n".join(context_parts)

    async def _generate_openrouter_stream(
        self,
        system_prompt: str,
        user_prompt: str
    ) -> AsyncGenerator[str, None]:
        """Generate streaming response using OpenRouter API with timeout."""
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]

        try:
            # Use asyncio.wait_for for timeout with OpenRouter
            stream = await asyncio.wait_for(
                self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    max_tokens=min(self.max_tokens, 800),  # Reasonable limit
                    temperature=self.temperature,
                    stream=True
                ),
                timeout=25.0  # 25 second timeout
            )

            # Process streaming events
            async for chunk in stream:
                if chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content
                # Add small yield to prevent blocking
                await asyncio.sleep(0)

        except asyncio.TimeoutError:
            logger.error("OpenRouter API timeout after 25 seconds")
            yield "Response generation timed out. Please try a shorter question."
        except Exception as e:
            logger.error(f"OpenRouter API error: {e}")
            yield f"Error generating response: {str(e)}"

    async def _generate_cohere_stream(
        self,
        system_prompt: str,
        user_prompt: str
    ) -> AsyncGenerator[str, None]:
        """Generate streaming response using Cohere API with timeout."""
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]

        try:
            # Use asyncio.wait_for with regular chat call (non-streaming for simplicity)
            response = await asyncio.wait_for(
                asyncio.to_thread(
                    self.client.chat,
                    model=self.model,
                    messages=messages,
                    max_tokens=min(self.max_tokens, 400),  # Reduced for speed
                    temperature=self.temperature
                ),
                timeout=15.0  # 15 second timeout
            )

            # Get the full response text
            if response and response.message and response.message.content:
                full_text = response.message.content[0].text
                # Yield the response character by character for streaming effect
                for i in range(0, len(full_text), 5):  # Yield chunks of 5 chars
                    chunk = full_text[i:i+5]
                    yield chunk
                    await asyncio.sleep(0.01)  # Small delay for streaming effect
            else:
                yield "I'm sorry, I couldn't generate a response based on the provided context."

        except asyncio.TimeoutError:
            logger.error("Cohere API timeout after 15 seconds")
            yield "Response generation timed out. Please try a shorter question."
        except Exception as e:
            logger.error(f"Cohere API error: {e}")
            yield f"Error generating response: {str(e)}"

    async def _async_generator(self, sync_gen):
        """Convert sync generator to async generator."""
        for item in sync_gen:
            yield item
            await asyncio.sleep(0)  # Yield control

    async def _async_generator_with_timeout(self, sync_gen, timeout):
        """Convert sync generator to async with timeout."""
        import asyncio

        async def producer():
            for item in sync_gen:
                yield item
                await asyncio.sleep(0)  # Yield control

        async for item in producer():
            yield item
            # Check if we're running too long
            await asyncio.sleep(0)

    async def _generate_openai_stream(
        self,
        system_prompt: str,
        user_prompt: str
    ) -> AsyncGenerator[str, None]:
        """Generate streaming response using OpenAI API."""
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]

        try:
            stream = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                stream=True
            )

            async for chunk in stream:
                if chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content

        except Exception as e:
            logger.error(f"OpenAI API error: {e}")
            raise

    async def health_check(self) -> Dict[str, Any]:
        """Check the health of the LLM service."""
        try:
            # Simple test call
            test_messages = [
                {"role": "user", "content": "Say 'Hello'"}
            ]

            if self.provider == "openrouter":
                try:
                    response = await asyncio.wait_for(
                        self.client.chat.completions.create(
                            model=self.model,
                            messages=test_messages,
                            max_tokens=10
                        ),
                        timeout=5.0  # 5 second timeout for health check
                    )

                    return {
                        "status": "healthy",
                        "provider": self.provider,
                        "model": self.model,
                        "test_response": response.choices[0].message.content[:50]
                    }
                except asyncio.TimeoutError:
                    return {
                        "status": "degraded",
                        "provider": self.provider,
                        "error": "Health check timeout"
                    }
            elif self.provider == "cohere":
                try:
                    # Use asyncio.to_thread for sync call
                    response = await asyncio.wait_for(
                        asyncio.to_thread(
                            self.client.chat,
                            model=self.model,
                            messages=test_messages,
                            max_tokens=10
                        ),
                        timeout=5.0  # 5 second timeout for health check
                    )

                    return {
                        "status": "healthy",
                        "provider": self.provider,
                        "model": self.model,
                        "test_response": response.message.content[0].text[:50]
                    }
                except asyncio.TimeoutError:
                    return {
                        "status": "degraded",
                        "provider": self.provider,
                        "error": "Health check timeout"
                    }
            elif self.provider == "openai":
                response = await self.client.chat.completions.create(
                    model=self.model,
                    messages=test_messages,
                    max_tokens=10
                )

                return {
                    "status": "healthy",
                    "provider": self.provider,
                    "model": self.model,
                    "test_response": response.choices[0].message.content[:50]
                }
        except Exception as e:
            return {
                "status": "unhealthy",
                "error": str(e),
                "provider": self.provider
            }


# Default configuration
DEFAULT_LLM_CONFIG = {
    "provider": "openrouter",
    "model": "mistralai/devstral-2512:free",
    "max_tokens": 1000,
    "temperature": 0.7
}


def create_llm_service(config: Optional[Dict[str, Any]] = None) -> LLMService:
    """Create an LLM service with the given configuration."""
    if config is None:
        config = DEFAULT_LLM_CONFIG

    return LLMService(**config)