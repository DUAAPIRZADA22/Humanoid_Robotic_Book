"""
OpenAI-based reranking service using semantic similarity.
Provides document reranking using OpenAI embeddings and similarity calculations.
"""

import os
import asyncio
import logging
import numpy as np
from typing import List, Dict, Any, Optional
from tenacity import retry, stop_after_attempt, wait_exponential
from .openai_embeddings import OpenAIEmbeddingService

logger = logging.getLogger(__name__)


class OpenAIReranker:
    """
    OpenAI-based reranking service using embedding similarity.
    """

    def __init__(
        self,
        api_key: str,
        model: str = "text-embedding-3-small",
        max_chunks_per_doc: int = 10,
        top_n: Optional[int] = None,
        timeout: int = 30,
        base_url: Optional[str] = None
    ):
        """
        Initialize the OpenAI reranker.

        Args:
            api_key: OpenAI API key
            model: Embedding model for similarity calculation
            max_chunks_per_doc: Maximum chunks per document
            top_n: Default number of top documents to return
            timeout: Request timeout in seconds
            base_url: Custom base URL for OpenAI
        """
        self.model = model
        self.max_chunks_per_doc = max_chunks_per_doc
        self.top_n = top_n
        self.timeout = timeout

        # Initialize OpenAI embedding service
        self.embedding_service = OpenAIEmbeddingService(
            api_key=api_key,
            model=model,
            timeout=timeout,
            base_url=base_url
        )

        logger.info(f"Initialized OpenAI reranker with model: {model}")

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10)
    )
    async def rerank(
        self,
        query: str,
        documents: List[Dict[str, Any]],
        top_n: Optional[int] = None,
        return_documents: bool = True,
        rank_fields: Optional[List[str]] = None
    ) -> List[Dict[str, Any]]:
        """
        Rerank documents based on relevance to query using OpenAI embeddings.

        Args:
            query: Search query string
            documents: List of documents to rerank
            top_n: Number of top documents to return
            return_documents: Whether to return document content
            rank_fields: Fields to use for ranking (if documents are dicts)

        Returns:
            List of reranked documents with relevance scores
        """
        if not documents:
            return []

        if not query or not query.strip():
            logger.warning("Empty query provided for reranking")
            return documents

        top_n = top_n or self.top_n

        try:
            # Prepare documents for embedding
            doc_texts = []
            original_docs = []

            for i, doc in enumerate(documents):
                if isinstance(doc, str):
                    doc_texts.append(doc)
                    original_docs.append({"text": doc, "original_index": i})
                elif isinstance(doc, dict):
                    # Extract text from document dict
                    if 'text' in doc:
                        doc_texts.append(doc['text'])
                        doc_copy = doc.copy()
                        doc_copy['original_index'] = i
                        original_docs.append(doc_copy)
                    elif 'content' in doc:
                        doc_texts.append(doc['content'])
                        doc_copy = doc.copy()
                        doc_copy['text'] = doc['content']
                        doc_copy['original_index'] = i
                        original_docs.append(doc_copy)
                    else:
                        # Use string representation
                        doc_text = str(doc)
                        doc_texts.append(doc_text)
                        original_docs.append({"text": doc_text, "original_index": i})
                else:
                    doc_texts.append(str(doc))
                    original_docs.append({"text": str(doc), "original_index": i})

            logger.debug(f"Reranking {len(doc_texts)} documents for query: {query[:50]}...")

            # Generate embeddings for query
            query_embedding = await self.embedding_service.embed_single_text(query)

            # Generate embeddings for all documents in batch
            doc_embeddings = await self.embedding_service.embed_texts(doc_texts)

            # Calculate similarity scores using cosine similarity
            similarities = []
            query_norm = np.linalg.norm(query_embedding)

            for doc_embedding in doc_embeddings:
                doc_norm = np.linalg.norm(doc_embedding)
                if query_norm > 0 and doc_norm > 0:
                    # Cosine similarity
                    similarity = np.dot(query_embedding, doc_embedding) / (query_norm * doc_norm)
                else:
                    similarity = 0.0
                similarities.append(similarity)

            # Combine original docs with similarity scores
            reranked_docs = []
            for i, (doc, similarity) in enumerate(zip(original_docs, similarities)):
                doc_copy = doc.copy()
                doc_copy['relevance_score'] = similarity
                doc_copy['rerank_index'] = i
                reranked_docs.append(doc_copy)

            # Sort by similarity score (descending)
            reranked_docs.sort(key=lambda x: x['relevance_score'], reverse=True)

            # Return top_n documents
            result = reranked_docs[:min(top_n, len(reranked_docs))]

            logger.info(f"Successfully reranked {len(result)} documents")
            return result

        except Exception as e:
            logger.error(f"Error during reranking: {str(e)}")
            # Return original documents on error
            return documents[:top_n] if top_n else documents

    async def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the reranker service.

        Returns:
            Health check result
        """
        try:
            # Test embedding service
            health = await self.embedding_service.health_check()

            if health.get("status") == "healthy":
                return {
                    "status": "healthy",
                    "provider": "openai",
                    "model": self.model,
                    "embedding_dimension": health.get("dimension"),
                    "method": "embedding_similarity"
                }
            else:
                return {
                    "status": "unhealthy",
                    "provider": "openai",
                    "error": f"Embedding service unhealthy: {health.get('error', 'Unknown error')}",
                    "model": self.model
                }
        except Exception as e:
            return {
                "status": "unhealthy",
                "provider": "openai",
                "error": str(e),
                "model": self.model
            }


def create_openai_reranker(config: Optional[Dict[str, Any]] = None) -> OpenAIReranker:
    """
    Create an OpenAI reranker with the given configuration.

    Args:
        config: Configuration dictionary

    Returns:
        OpenAIReranker instance
    """
    if config is None:
        config = {}

    return OpenAIReranker(
        api_key=config.get("api_key", os.getenv("OPENAI_API_KEY")),
        model=config.get("model", "text-embedding-3-small"),
        max_chunks_per_doc=config.get("max_chunks_per_doc", 10),
        top_n=config.get("top_n", 5),
        timeout=config.get("timeout", 30),
        base_url=config.get("base_url", os.getenv("OPENAI_BASE_URL"))
    )