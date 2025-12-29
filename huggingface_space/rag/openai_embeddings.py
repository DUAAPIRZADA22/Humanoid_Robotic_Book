"""
OpenAI embedding service implementation using OpenAI API.
Provides text embedding generation with async support and proper handling.
"""

import os
import asyncio
import logging
from typing import List, Optional, Dict, Any
import openai
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)


class OpenAIEmbeddingService:
    """
    OpenAI embedding service using the OpenAI Python SDK.
    """

    def __init__(
        self,
        api_key: str,
        model: str = "text-embedding-3-small",
        batch_size: int = 100,
        max_retries: int = 3,
        timeout: int = 30,
        base_url: Optional[str] = None
    ):
        """
        Initialize the OpenAI embedding service.

        Args:
            api_key: OpenAI API key
            model: Embedding model name (default: text-embedding-3-small)
            batch_size: Number of texts to embed in one batch
            max_retries: Maximum number of retry attempts
            timeout: Request timeout in seconds
            base_url: Custom base URL (for OpenRouter compatibility)
        """
        self.api_key = api_key
        self.model = model
        self.batch_size = batch_size
        self.max_retries = max_retries
        self.timeout = timeout

        # Initialize OpenAI client
        self.client = openai.AsyncOpenAI(
            api_key=api_key,
            base_url=base_url
        )

        logger.info(f"Initialized OpenAI embedding service with model: {model}")

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10)
    )
    async def embed_texts(
        self,
        texts: List[str],
        input_type: str = "search_document"
    ) -> List[List[float]]:
        """
        Embed a list of texts using OpenAI's embedding API.

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
                response = await asyncio.wait_for(
                    self.client.embeddings.create(
                        model=self.model,
                        input=batch
                    ),
                    timeout=self.timeout
                )

                # Extract embeddings from response
                batch_embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(batch_embeddings)

                logger.debug(f"Successfully embedded batch {i//self.batch_size + 1}")

            except asyncio.TimeoutError:
                logger.error(f"Embedding timeout for batch {i//self.batch_size + 1}")
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
        # OpenAI text-embedding-3-small returns 1536 dimensions
        # text-embedding-3-large returns 3072 dimensions
        if self.model == "text-embedding-3-small":
            return 1536
        elif self.model == "text-embedding-3-large":
            return 3072
        elif self.model == "text-embedding-ada-002":
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
                "provider": "openai",
                "model": self.model,
                "dimension": len(test_embedding) if test_embedding else 0,
                "test_embedding_length": len(test_embedding) if test_embedding else 0
            }
        except Exception as e:
            return {
                "status": "unhealthy",
                "provider": "openai",
                "error": str(e),
                "model": self.model
            }


def create_openai_embedding_service(config: Optional[Dict[str, Any]] = None) -> OpenAIEmbeddingService:
    """
    Create an OpenAI embedding service with the given configuration.

    Args:
        config: Configuration dictionary

    Returns:
        OpenAIEmbeddingService instance
    """
    if config is None:
        config = {}

    return OpenAIEmbeddingService(
        api_key=config.get("api_key", os.getenv("OPENAI_API_KEY")),
        model=config.get("model", "text-embedding-3-small"),
        batch_size=config.get("batch_size", 100),
        max_retries=config.get("max_retries", 3),
        timeout=config.get("timeout", 30),
        base_url=config.get("base_url", os.getenv("OPENAI_BASE_URL"))
    )