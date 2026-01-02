"""
Cohere reranking service implementation using latest Cohere API.
Provides document reranking with improved relevance scoring.
"""

import os
import asyncio
import logging
from typing import List, Dict, Any, Optional
import cohere
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)


class CohereReranker:
    """
    Cohere reranking service using the latest Cohere Python SDK.
    """

    def __init__(
        self,
        api_key: str,
        model: str = "rerank-v3.5",
        max_chunks_per_doc: int = 10,
        top_n: int = 10,
        timeout: int = 30
    ):
        """
        Initialize the Cohere reranking service.

        Args:
            api_key: Cohere API key
            model: Reranking model to use
            max_chunks_per_doc: Maximum chunks per document for complex documents
            top_n: Default number of top results to return
            timeout: Request timeout in seconds
        """
        self.api_key = api_key
        self.model = model
        self.max_chunks_per_doc = max_chunks_per_doc
        self.top_n = top_n
        self.timeout = timeout

        # Initialize Cohere client
        self.client = cohere.Client(
            api_key=api_key,
            timeout=timeout
        )

        logger.info(f"Initialized Cohere reranker with model: {model}")

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
        Rerank documents based on relevance to query using latest Cohere API.

        Args:
            query: Search query
            documents: List of documents to rerank (can be strings or dicts with 'text' field)
            top_n: Number of top results to return
            return_documents: Whether to return document text in results
            rank_fields: List of fields to consider for ranking (for complex documents)

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
            # Prepare documents for Cohere API
            cohere_docs = []
            for i, doc in enumerate(documents):
                if isinstance(doc, str):
                    cohere_docs.append(doc)
                elif isinstance(doc, dict):
                    # If document is a dict, extract text or use the whole doc
                    if 'text' in doc:
                        cohere_docs.append(doc['text'])
                    else:
                        # Create a string representation for complex docs
                        doc_text = str(doc.get('content', str(doc)))
                        cohere_docs.append(doc_text)
                else:
                    cohere_docs.append(str(doc))

            logger.debug(f"Reranking {len(cohere_docs)} documents for query: {query[:50]}...")

            # Use asyncio to run synchronous rerank in thread pool with timeout
            import asyncio
            response = await asyncio.wait_for(
                asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.client.rerank(
                        model=self.model,
                        query=query,
                        documents=cohere_docs,
                        top_n=min(top_n, len(cohere_docs))
                    )
                ),
                timeout=10.0  # 10 second timeout for reranking
            )

            # Convert results back to our format
            reranked_results = []
            for result in response.results:
                original_doc = documents[result.index]

                if isinstance(original_doc, dict):
                    # Preserve original document structure
                    reranked_doc = original_doc.copy()
                    reranked_doc['relevance_score'] = result.relevance_score
                    reranked_doc['rerank_index'] = result.index
                else:
                    # Create new document structure
                    reranked_doc = {
                        'text': original_doc,
                        'relevance_score': result.relevance_score,
                        'rerank_index': result.index,
                        'metadata': {}
                    }

                reranked_results.append(reranked_doc)

            logger.info(f"Successfully reranked {len(reranked_results)} documents")
            return reranked_results

        except Exception as e:
            logger.error(f"Failed to rerank documents: {str(e)}")
            # Return original documents on error
            return documents

    async def batch_rerank(
        self,
        queries: List[str],
        document_lists: List[List[Dict[str, Any]]],
        top_n: Optional[int] = None
    ) -> List[List[Dict[str, Any]]]:
        """
        Rerank multiple query-document pairs.

        Args:
            queries: List of search queries
            document_lists: List of document lists (one per query)
            top_n: Number of top results to return per query

        Returns:
            List of reranked document lists
        """
        if len(queries) != len(document_lists):
            raise ValueError("Number of queries must match number of document lists")

        results = []
        for query, docs in zip(queries, document_lists):
            reranked = await self.rerank(query, docs, top_n)
            results.append(reranked)

        return results

    async def hybrid_rerank(
        self,
        query: str,
        documents: List[Dict[str, Any]],
        vector_scores: List[float],
        rerank_weight: float = 0.7,
        vector_weight: float = 0.3
    ) -> List[Dict[str, Any]]:
        """
        Combine reranking scores with vector similarity scores.

        Args:
            query: Search query
            documents: Documents to rerank
            vector_scores: Vector similarity scores
            rerank_weight: Weight for reranking scores
            vector_weight: Weight for vector scores

        Returns:
            Documents with combined relevance scores
        """
        # Get reranked results
        reranked_docs = await self.rerank(query, documents, return_documents=True)

        # Combine scores
        for i, doc in enumerate(reranked_docs):
            rerank_score = doc.get('relevance_score', 0.0)
            vector_score = vector_scores[doc.get('rerank_index', i)] if i < len(vector_scores) else 0.0

            # Normalize scores to 0-1 range
            normalized_rerank = min(rerank_score, 1.0)
            normalized_vector = min(vector_score, 1.0)

            # Calculate combined score
            combined_score = (
                rerank_weight * normalized_rerank +
                vector_weight * normalized_vector
            )

            doc['combined_score'] = combined_score
            doc['vector_score'] = vector_score

        # Sort by combined score
        reranked_docs.sort(key=lambda x: x['combined_score'], reverse=True)

        return reranked_docs

    async def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the reranking service.

        Returns:
            Health status information
        """
        try:
            # Test with a simple reranking task
            test_docs = [
                "This is about artificial intelligence",
                "This is about machine learning",
                "This is about cooking recipes"
            ]

            result = await self.rerank("AI technology", test_docs, top_n=2)

            return {
                "status": "healthy",
                "model": self.model,
                "top_n": self.top_n,
                "test_results": len(result)
            }
        except Exception as e:
            return {
                "status": "unhealthy",
                "error": str(e),
                "model": self.model
            }

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the current reranking model.

        Returns:
            Model information
        """
        model_info = {
            "rerank-v3.5": {
                "description": "Latest reranking model with improved accuracy",
                "max_documents": 1000,
                "max_chunks_per_doc": 10
            },
            "rerank-english-v3.0": {
                "description": "English reranking model",
                "max_documents": 1000,
                "max_chunks_per_doc": 10
            },
            "rerank-multilingual-v3.0": {
                "description": "Multilingual reranking model",
                "max_documents": 1000,
                "max_chunks_per_doc": 10
            }
        }

        return {
            "model": self.model,
            "info": model_info.get(self.model, {}),
            "top_n": self.top_n,
            "max_chunks_per_doc": self.max_chunks_per_doc
        }


# For backward compatibility
class Reranker(CohereReranker):
    """Alias for backward compatibility"""
    pass