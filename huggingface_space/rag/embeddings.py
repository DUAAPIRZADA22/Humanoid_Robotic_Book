"""
Cohere embedding service implementation.
Handles batching, rate limiting, and error recovery.
"""

import os
import asyncio
import time
from typing import List, Optional, Dict, Any
import logging
import aiohttp
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

logger = logging.getLogger(__name__)


class CohereEmbeddingService:
    """
    Cohere embedding service with async support and intelligent batching.
    """

    def __init__(
        self,
        api_key: str,
        model: str = "embed-english-v3.0",
        batch_size: int = 100,
        max_retries: int = 3,
        rate_limit_delay: float = 0.1
    ):
        """
        Initialize the embedding service.

        Args:
            api_key: Cohere API key
            model: Embedding model to use
            batch_size: Number of texts to embed in one request
            max_retries: Maximum number of retries for failed requests
            rate_limit_delay: Delay between requests to avoid rate limiting
        """
        self.api_key = api_key
        self.model = model
        self.batch_size = batch_size
        self.max_retries = max_retries
        self.rate_limit_delay = rate_limit_delay
        self.base_url = "https://api.cohere.ai/v1"

        # Model dimensions mapping
        self.model_dimensions = {
            "embed-english-v3.0": 1024,
            "embed-multilingual-v3.0": 1024,
            "embed-english-light-v3.0": 384,
            "embed-multilingual-light-v3.0": 384
        }

        if model not in self.model_dimensions:
            raise ValueError(f"Unsupported model: {model}")

        self.dimension = self.model_dimensions[model]

    async def get_embeddings(
        self,
        texts: List[str],
        input_type: str = "search_document"
    ) -> List[List[float]]:
        """
        Get embeddings for a list of texts.

        Args:
            texts: List of texts to embed
            input_type: Type of input for embedding optimization

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        # Filter out empty texts
        valid_texts = [(i, text) for i, text in enumerate(texts) if text and text.strip()]
        if not valid_texts:
            return [None] * len(texts)

        # Process in batches
        all_embeddings = [None] * len(texts)
        batches = self._create_batches(valid_texts)

        async with aiohttp.ClientSession() as session:
            for batch in batches:
                batch_texts = [text for _, text in batch]
                batch_indices = [i for i, _ in batch]

                try:
                    embeddings = await self._embed_batch(
                        session,
                        batch_texts,
                        input_type
                    )

                    # Place embeddings in correct positions
                    for i, embedding in zip(batch_indices, embeddings):
                        all_embeddings[i] = embedding

                except Exception as e:
                    logger.error(f"Failed to embed batch: {str(e)}")
                    raise

        # Validate all embeddings were created
        if None in all_embeddings:
            missing = sum(1 for e in all_embeddings if e is None)
            logger.warning(f"Failed to embed {missing} texts")

        return [e for e in all_embeddings if e is not None]

    def _create_batches(self, indexed_texts: List[tuple]) -> List[List[tuple]]:
        """
        Create batches of texts for processing.
        """
        batches = []
        current_batch = []
        current_length = 0

        for index, text in indexed_texts:
            # Check if adding this text would exceed batch size
            text_length = len(text)
            if (current_batch and
                (len(current_batch) >= self.batch_size or
                 current_length + text_length > 8000)):  # Character limit per batch
                batches.append(current_batch)
                current_batch = [(index, text)]
                current_length = text_length
            else:
                current_batch.append((index, text))
                current_length += text_length

        if current_batch:
            batches.append(current_batch)

        return batches

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10),
        retry=retry_if_exception_type((aiohttp.ClientError, asyncio.TimeoutError))
    )
    async def _embed_batch(
        self,
        session: aiohttp.ClientSession,
        texts: List[str],
        input_type: str
    ) -> List[List[float]]:
        """
        Embed a batch of texts with retry logic.
        """
        # Rate limiting
        await asyncio.sleep(self.rate_limit_delay)

        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
            "Accept": "application/json"
        }

        payload = {
            "model": self.model,
            "texts": texts,
            "input_type": input_type
        }

        async with session.post(
            f"{self.base_url}/embed",
            headers=headers,
            json=payload,
            timeout=30
        ) as response:
            if response.status == 200:
                data = await response.json()
                embeddings = data.get("embeddings", [])

                # Validate embeddings
                if len(embeddings) != len(texts):
                    raise ValueError(f"Expected {len(texts)} embeddings, got {len(embeddings)}")

                for embedding in embeddings:
                    if len(embedding) != self.dimension:
                        raise ValueError(f"Expected dimension {self.dimension}, got {len(embedding)}")

                return embeddings
            else:
                error_text = await response.text()
                error_data = await response.json() if response.content_type == 'application/json' else {}

                # Handle specific error cases
                if response.status == 429:
                    # Rate limit exceeded
                    retry_after = response.headers.get('Retry-After', '5')
                    wait_time = float(retry_after)
                    logger.warning(f"Rate limit hit, waiting {wait_time} seconds")
                    await asyncio.sleep(wait_time)
                    raise aiohttp.ClientError(f"Rate limit exceeded: {error_data}")
                elif response.status == 400:
                    # Bad request
                    raise ValueError(f"Invalid request: {error_data}")
                elif response.status == 401:
                    raise PermissionError(f"Invalid API key: {error_data}")
                elif response.status >= 500:
                    # Server error
                    raise aiohttp.ClientError(f"Server error: {error_data}")
                else:
                    raise aiohttp.ClientError(f"API error {response.status}: {error_text}")

    async def get_embedding(
        self,
        text: str,
        input_type: str = "search_query"
    ) -> List[float]:
        """
        Get embedding for a single text.

        Args:
            text: Text to embed
            input_type: Type of input for embedding optimization

        Returns:
            Embedding vector
        """
        embeddings = await self.get_embeddings([text], input_type)
        return embeddings[0] if embeddings else None

    async def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the embedding service.
        """
        try:
            test_text = "Health check"
            embedding = await self.get_embedding(test_text)

            return {
                "status": "healthy",
                "model": self.model,
                "dimension": self.dimension,
                "test_embedding_shape": len(embedding) if embedding else None
            }
        except Exception as e:
            return {
                "status": "unhealthy",
                "error": str(e),
                "model": self.model
            }

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the current model.
        """
        return {
            "model": self.model,
            "dimension": self.dimension,
            "batch_size": self.batch_size,
            "max_retries": self.max_retries,
            "rate_limit_delay": self.rate_limit_delay
        }


class EmbeddingCache:
    """
    Simple in-memory cache for embeddings to reduce API calls.
    """

    def __init__(self, max_size: int = 10000):
        """
        Initialize the cache.

        Args:
            max_size: Maximum number of cached embeddings
        """
        self.max_size = max_size
        self.cache: Dict[str, List[float]] = {}
        self.access_times: Dict[str, float] = {}

    def get(self, text: str) -> Optional[List[float]]:
        """
        Get cached embedding for text.
        """
        import hashlib
        key = hashlib.sha256(text.encode()).hexdigest()

        if key in self.cache:
            self.access_times[key] = time.time()
            return self.cache[key]

        return None

    def set(self, text: str, embedding: List[float]) -> None:
        """
        Cache embedding for text.
        """
        import hashlib
        key = hashlib.sha256(text.encode()).hexdigest()

        # Evict if cache is full
        if len(self.cache) >= self.max_size:
            self._evict_lru()

        self.cache[key] = embedding
        self.access_times[key] = time.time()

    def _evict_lru(self) -> None:
        """
        Evict least recently used embedding.
        """
        if not self.cache:
            return

        # Find least recently used key
        lru_key = min(self.access_times.keys(), key=lambda k: self.access_times[k])

        # Remove from cache
        del self.cache[lru_key]
        del self.access_times[lru_key]

    def clear(self) -> None:
        """
        Clear all cached embeddings.
        """
        self.cache.clear()
        self.access_times.clear()

    def get_stats(self) -> Dict[str, Any]:
        """
        Get cache statistics.
        """
        return {
            "size": len(self.cache),
            "max_size": self.max_size,
            "usage": len(self.cache) / self.max_size
        }