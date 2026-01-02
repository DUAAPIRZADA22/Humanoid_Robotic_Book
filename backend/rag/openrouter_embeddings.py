"""
OpenRouter embedding service implementation using OpenRouter API.
Provides text embedding generation with async support and proper handling.
"""

import os
import asyncio
import logging
from typing import List, Optional, Dict, Any
import openai
import httpx
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

logger = logging.getLogger(__name__)


class OpenRouterEmbeddingService:
    """
    OpenRouter embedding service using the OpenRouter API with OpenAI-compatible interface.
    """

    def __init__(
        self,
        api_key: str,
        model: str = "openai/text-embedding-3-small",
        batch_size: int = 100,
        max_retries: int = 3,
        timeout: int = 60,
        base_url: str = "https://openrouter.ai/api/v1"
    ):
        """
        Initialize the OpenRouter embedding service.

        Args:
            api_key: OpenRouter API key
            model: Embedding model name (OpenAI compatible models)
            batch_size: Number of texts to embed in one batch
            max_retries: Maximum number of retry attempts
            timeout: Request timeout in seconds (default 60)
            base_url: OpenRouter base URL
        """
        self.api_key = api_key
        self.model = model
        self.batch_size = batch_size
        self.max_retries = max_retries
        self.timeout = timeout

        # Create httpx client with proper timeout configuration
        # Reduced timeout to 15s to work with circuit breaker - prevents long hangs
        http_client = httpx.AsyncClient(
            timeout=httpx.Timeout(timeout=15.0, connect=10.0),
            limits=httpx.Limits(max_keepalive_connections=5, max_connections=10)
        )

        # Initialize OpenRouter client using OpenAI SDK with custom httpx client
        self.client = openai.AsyncOpenAI(
            api_key=api_key,
            base_url=base_url,
            http_client=http_client
        )

        logger.info(f"Initialized OpenRouter embedding service with model: {model}, timeout: {timeout}s")

    @retry(
        stop=stop_after_attempt(2),  # Reduced from 3 to prevent long hangs
        wait=wait_exponential(multiplier=1, min=1, max=3),  # Reduced wait times
        retry=retry_if_exception_type((httpx.TimeoutException, httpx.NetworkError, openai.APITimeoutError))
    )
    async def embed_texts(
        self,
        texts: List[str],
        input_type: str = "search_document"
    ) -> List[List[float]]:
        """
        Embed a list of texts using OpenRouter's embedding API.

        Args:
            texts: List of text strings to embed
            input_type: Type of input (for compatibility with existing code)

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        logger.debug(f"Embedding {len(texts)} texts with model {self.model}")

        # Process in batches to avoid rate limits
        all_embeddings = []

        for i in range(0, len(texts), self.batch_size):
            batch = texts[i:i + self.batch_size]

            try:
                # httpx timeout handles the timeout internally now
                response = await self.client.embeddings.create(
                    model=self.model,
                    input=batch
                )

                # Extract embeddings from response
                batch_embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(batch_embeddings)

                logger.debug(f"Successfully embedded batch {i//self.batch_size + 1}")

                # Add small delay to avoid rate limiting
                if i + self.batch_size < len(texts):
                    await asyncio.sleep(0.1)

            except (httpx.TimeoutException, openai.APITimeoutError) as e:
                logger.warning(f"Timeout embedding batch {i//self.batch_size + 1}, retrying...: {str(e)}")
                raise
            except httpx.NetworkError as e:
                logger.warning(f"Network error embedding batch {i//self.batch_size + 1}, retrying...: {str(e)}")
                raise
            except openai.RateLimitError as e:
                logger.warning(f"Rate limit hit, waiting before retry...: {str(e)}")
                await asyncio.sleep(5)  # Wait longer for rate limits
                raise
            except Exception as e:
                logger.error(f"Error embedding batch {i//self.batch_size + 1}: {str(e)}")
                raise

        logger.info(f"Successfully generated {len(all_embeddings)} embeddings")
        return all_embeddings

    async def embed_single_text(self, text: str) -> List[float]:
        """
        Embed a single text string.

        Args:
            text: Text string to embed

        Returns:
            Embedding vector
        """
        if not text or not text.strip():
            raise ValueError("Text cannot be empty")

        embeddings = await self.embed_texts([text])
        return embeddings[0] if embeddings else []

    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of the embedding vectors.

        Returns:
            Embedding dimension
        """
        # For OpenAI text-embedding-3-small/large via OpenRouter
        if "text-embedding-3-small" in self.model:
            return 1536
        elif "text-embedding-3-large" in self.model:
            return 3072
        elif "text-embedding-ada-002" in self.model:
            return 1536
        else:
            # Default to 1536 for unknown models
            return 1536

    async def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the embedding service.

        Returns:
            Health check result
        """
        try:
            # Test embedding a simple text
            test_embedding = await self.embed_single_text("Health check test")

            return {
                "status": "healthy",
                "provider": "openrouter",
                "model": self.model,
                "dimension": len(test_embedding) if test_embedding else 0,
                "test_embedding_length": len(test_embedding) if test_embedding else 0
            }
        except Exception as e:
            return {
                "status": "unhealthy",
                "provider": "openrouter",
                "error": str(e),
                "model": self.model
            }


def create_openrouter_embedding_service(config: Optional[Dict[str, Any]] = None) -> OpenRouterEmbeddingService:
    """
    Create an OpenRouter embedding service with the given configuration.

    Args:
        config: Configuration dictionary

    Returns:
        OpenRouterEmbeddingService instance
    """
    if config is None:
        config = {}

    return OpenRouterEmbeddingService(
        api_key=config.get("api_key", os.getenv("OPENROUTER_API_KEY")),
        model=config.get("model", "openai/text-embedding-3-small"),
        batch_size=config.get("batch_size", 100),
        max_retries=config.get("max_retries", 3),
        timeout=config.get("timeout", 60),
        base_url=config.get("base_url", os.getenv("OPENROUTER_BASE_URL"))
    )