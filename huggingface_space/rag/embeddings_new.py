"""
Cohere embedding service implementation using latest Cohere API.
Provides text embedding generation with async support and proper input_type handling.
"""

import os
import asyncio
import logging
from typing import List, Optional, Dict, Any
import cohere
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)


class CohereEmbeddingService:
    """
    Cohere embedding service using the latest Cohere Python SDK.
    """

    def __init__(
        self,
        api_key: str,
        model: str = "embed-english-v3.0",
        batch_size: int = 96,
        max_retries: int = 3,
        timeout: int = 30
    ):
        """
        Initialize the Cohere embedding service.

        Args:
            api_key: Cohere API key
            model: Embedding model to use
            batch_size: Maximum batch size for embedding requests
            max_retries: Maximum number of retries for failed requests
            timeout: Request timeout in seconds
        """
        self.api_key = api_key
        self.model = model
        self.batch_size = batch_size
        self.max_retries = max_retries
        self.timeout = timeout

        # Initialize Cohere client
        self.client = cohere.Client(
            api_key=api_key,
            timeout=timeout
        )

        logger.info(f"Initialized Cohere embedding service with model: {model}")

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10)
    )
    async def get_embeddings(
        self,
        texts: List[str],
        input_type: str = "search_document"
    ) -> List[List[float]]:
        """
        Get embeddings for a list of texts using Cohere's latest API.

        Args:
            texts: List of text strings to embed
            input_type: Type of input ("search_document", "search_query", "classification", "clustering")

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        # Filter out empty or None texts
        valid_texts = [text for text in texts if text and text.strip()]

        if not valid_texts:
            logger.warning("No valid texts provided for embedding")
            return []

        try:
            # Process in batches according to Cohere's limits (max 96 texts per request)
            all_embeddings = []

            for i in range(0, len(valid_texts), self.batch_size):
                batch = valid_texts[i:i + self.batch_size]

                logger.debug(f"Embedding batch of {len(batch)} texts")

                # Use Cohere's synchronous client in async context
                # The new Cohere API handles batching internally
                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type=input_type
                )

                # Extract embeddings from response - they are directly available as lists
                try:
                    batch_embeddings = []

                    # The Cohere API returns embeddings directly as a list of float arrays
                    for i, embedding in enumerate(response.embeddings):
                        if isinstance(embedding, list):
                            # Direct list of floats - this is what we expect
                            batch_embeddings.append(embedding)
                        elif hasattr(embedding, 'tolist'):
                            # Handle numpy arrays or similar objects
                            batch_embeddings.append(embedding.tolist())
                        else:
                            # Try to convert to list
                            batch_embeddings.append(list(embedding))

                except Exception as e:
                    logger.error(f"Error extracting embeddings: {e}")
                    logger.error(f"Response type: {type(response.embeddings)}")
                    logger.error(f"First embedding: {response.embeddings[0] if hasattr(response.embeddings, '__getitem__') else 'N/A'}")
                    batch_embeddings = []

                all_embeddings.extend(batch_embeddings)

                # Small delay to respect rate limits
                if i + self.batch_size < len(valid_texts):
                    await asyncio.sleep(0.1)

            logger.info(f"Successfully generated {len(all_embeddings)} embeddings")
            return all_embeddings

        except Exception as e:
            logger.error(f"Failed to generate embeddings: {str(e)}")
            raise

    async def get_single_embedding(
        self,
        text: str,
        input_type: str = "search_query"
    ) -> List[float]:
        """
        Get embedding for a single text.

        Args:
            text: Text to embed
            input_type: Type of input

        Returns:
            Single embedding vector
        """
        embeddings = await self.get_embeddings([text], input_type)
        return embeddings[0] if embeddings else []

    async def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the embedding service.

        Returns:
            Health status information
        """
        try:
            # Test with a simple embedding
            test_embedding = await self.get_single_embedding("test", "search_query")

            return {
                "status": "healthy",
                "model": self.model,
                "embedding_dimension": len(test_embedding) if test_embedding else 0,
                "batch_size": self.batch_size
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

        Returns:
            Model information
        """
        model_info = {
            "embed-english-v3.0": {
                "dimensions": 1024,
                "languages": ["English"],
                "max_input_tokens": 2048,
                "description": "English embedding model optimized for search"
            },
            "embed-multilingual-v3.0": {
                "dimensions": 1024,
                "languages": ["Multiple"],
                "max_input_tokens": 2048,
                "description": "Multilingual embedding model"
            },
            "embed-english-light-v3.0": {
                "dimensions": 384,
                "languages": ["English"],
                "max_input_tokens": 2048,
                "description": "Lightweight English embedding model"
            }
        }

        return {
            "model": self.model,
            "info": model_info.get(self.model, {}),
            "batch_size": self.batch_size
        }


# For backward compatibility
class EmbeddingService(CohereEmbeddingService):
    """Alias for backward compatibility"""
    pass