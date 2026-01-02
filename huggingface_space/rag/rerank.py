"""
Cohere reranking service for improving retrieval relevance.
Provides intelligent reranking of search results.
"""

import asyncio
import logging
from typing import List, Dict, Any, Optional
import aiohttp
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)


class CohereReranker:
    """
    Cohere reranking service for improving search result relevance.
    """

    def __init__(
        self,
        api_key: str,
        model: str = "rerank-english-v3.0",
        max_retries: int = 3,
        rate_limit_delay: float = 0.1
    ):
        """
        Initialize the reranker.

        Args:
            api_key: Cohere API key
            model: Reranking model to use
            max_retries: Maximum number of retries for failed requests
            rate_limit_delay: Delay between requests to avoid rate limiting
        """
        self.api_key = api_key
        self.model = model
        self.max_retries = max_retries
        self.rate_limit_delay = rate_limit_delay
        self.base_url = "https://api.cohere.ai/v1"

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=4, max=10)
    )
    async def rerank(
        self,
        query: str,
        documents: List[Dict[str, Any]],
        top_k: Optional[int] = None,
        return_documents: bool = True
    ) -> List[Dict[str, Any]]:
        """
        Rerank documents based on query relevance.

        Args:
            query: Query string
            documents: List of documents to rerank
            top_k: Number of top results to return
            return_documents: Whether to return document content

        Returns:
            Reranked list of documents with relevance scores
        """
        if not documents:
            return []

        # Extract document texts
        doc_texts = [doc.get("text", "") for doc in documents]
        top_k = top_k or len(documents)

        # Rate limiting
        await asyncio.sleep(self.rate_limit_delay)

        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
            "Accept": "application/json"
        }

        payload = {
            "model": self.model,
            "query": query,
            "documents": doc_texts,
            "top_k": top_k,
            "return_documents": return_documents
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"{self.base_url}/rerank",
                headers=headers,
                json=payload,
                timeout=30
            ) as response:
                if response.status == 200:
                    data = await response.json()
                    return self._process_rerank_response(data, documents)
                else:
                    error_text = await response.text()
                    error_data = await response.json() if response.content_type == 'application/json' else {}

                    # Handle specific error cases
                    if response.status == 429:
                        retry_after = response.headers.get('Retry-After', '5')
                        wait_time = float(retry_after)
                        logger.warning(f"Rate limit hit, waiting {wait_time} seconds")
                        await asyncio.sleep(wait_time)
                        raise aiohttp.ClientError(f"Rate limit exceeded: {error_data}")
                    elif response.status == 400:
                        raise ValueError(f"Invalid request: {error_data}")
                    elif response.status == 401:
                        raise PermissionError(f"Invalid API key: {error_data}")
                    elif response.status >= 500:
                        raise aiohttp.ClientError(f"Server error: {error_data}")
                    else:
                        raise aiohttp.ClientError(f"API error {response.status}: {error_text}")

    def _process_rerank_response(
        self,
        response_data: Dict[str, Any],
        original_documents: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """
        Process the rerank API response and merge with original documents.
        """
        results = response_data.get("results", [])
        reranked_docs = []

        for result in results:
            index = result.get("index")
            if index is not None and index < len(original_documents):
                original_doc = original_documents[index].copy()

                # Add reranking information
                original_doc["rerank_score"] = result.get("relevance_score", 0.0)
                original_doc["rerank_index"] = index

                reranked_docs.append(original_doc)

        # If no results or API error, return original documents
        if not reranked_docs:
            logger.warning("Reranking failed, returning original order")
            return original_documents

        return reranked_docs

    async def batch_rerank(
        self,
        queries: List[str],
        documents_list: List[List[Dict[str, Any]]],
        top_k: Optional[int] = None
    ) -> List[List[Dict[str, Any]]]:
        """
        Rerank multiple query-document pairs concurrently.

        Args:
            queries: List of query strings
            documents_list: List of document lists for each query
            top_k: Number of top results to return per query

        Returns:
            List of reranked document lists
        """
        if not queries or not documents_list:
            return []

        if len(queries) != len(documents_list):
            raise ValueError("Number of queries must match number of document lists")

        # Create tasks for concurrent reranking
        tasks = [
            self.rerank(query=query, documents=docs, top_k=top_k)
            for query, docs in zip(queries, documents_list)
        ]

        # Wait for all tasks to complete
        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Handle exceptions
        processed_results = []
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                logger.error(f"Reranking query {i} failed: {str(result)}")
                processed_results.append(documents_list[i])  # Return original docs
            else:
                processed_results.append(result)

        return processed_results

    async def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the reranking model.
        """
        return {
            "model": self.model,
            "max_retries": self.max_retries,
            "rate_limit_delay": self.rate_limit_delay,
            "base_url": self.base_url
        }

    async def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the reranking service.
        """
        try:
            # Test with a simple query and documents
            test_query = "What is robotics?"
            test_docs = [
                {"text": "Robotics is the study of robots."},
                {"text": "Computer science involves programming."}
            ]

            result = await self.rerank(
                query=test_query,
                documents=test_docs,
                top_k=2
            )

            if result and len(result) > 0:
                return {
                    "status": "healthy",
                    "model": self.model,
                    "test_results": len(result),
                    "top_score": max(doc.get("rerank_score", 0) for doc in result)
                }
            else:
                return {
                    "status": "unhealthy",
                    "error": "No results from test query"
                }

        except Exception as e:
            return {
                "status": "unhealthy",
                "error": str(e),
                "model": self.model
            }


class HybridReranker:
    """
    Hybrid reranker combining multiple signals for better ranking.
    """

    def __init__(
        self,
        cohere_reranker: Optional[CohereReranker] = None,
        weights: Optional[Dict[str, float]] = None
    ):
        """
        Initialize the hybrid reranker.

        Args:
            cohere_reranker: Cohere reranking service
            weights: Weights for different ranking signals
        """
        self.cohere_reranker = cohere_reranker
        self.weights = weights or {
            "similarity_score": 0.4,
            "rerank_score": 0.4,
            "freshness_score": 0.1,
            "authority_score": 0.1
        }

    async def hybrid_rerank(
        self,
        query: str,
        documents: List[Dict[str, Any]],
        top_k: Optional[int] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform hybrid reranking using multiple signals.

        Args:
            query: Query string
            documents: Documents to rerank
            top_k: Number of top results to return

        Returns:
            Reranked documents
        """
        if not documents:
            return []

        # Step 1: Apply Cohere reranking if available
        if self.cohere_reranker:
            documents = await self.cohere_rerank.rerank(
                query=query,
                documents=documents
            )

        # Step 2: Calculate hybrid scores
        scored_docs = []
        for doc in documents:
            score = self._calculate_hybrid_score(doc)
            doc["hybrid_score"] = score
            scored_docs.append(doc)

        # Step 3: Sort by hybrid score
        scored_docs.sort(key=lambda x: x["hybrid_score"], reverse=True)

        # Step 4: Return top-k results
        if top_k:
            scored_docs = scored_docs[:top_k]

        return scored_docs

    def _calculate_hybrid_score(self, document: Dict[str, Any]) -> float:
        """
        Calculate a hybrid score using multiple signals.
        """
        score = 0.0

        # Similarity score (from vector search)
        similarity_score = document.get("score", 0.0)
        score += self.weights["similarity_score"] * similarity_score

        # Rerank score (from Cohere)
        rerank_score = document.get("rerank_score", 0.0)
        if rerank_score > 0:
            score += self.weights["rerank_score"] * rerank_score

        # Freshness score (based on metadata if available)
        freshness_score = self._calculate_freshness_score(document)
        score += self.weights["freshness_score"] * freshness_score

        # Authority score (based on source or other metadata)
        authority_score = self._calculate_authority_score(document)
        score += self.weights["authority_score"] * authority_score

        return score

    def _calculate_freshness_score(self, document: Dict[str, Any]) -> float:
        """
        Calculate freshness score based on document metadata.
        """
        # This is a simplified implementation
        # In practice, you'd use creation/modification dates
        metadata = document.get("metadata", {})

        # Check if document has date information
        if "date" in metadata or "created_at" in metadata:
            return 1.0  # Assume recent if date is available

        # Default score
        return 0.5

    def _calculate_authority_score(self, document: Dict[str, Any]) -> float:
        """
        Calculate authority score based on source or other signals.
        """
        metadata = document.get("metadata", {})
        source = metadata.get("source", "")

        # Check for authoritative sources
        authoritative_sources = [
            "wikipedia.org",
            "arxiv.org",
            "ieee.org",
            "acm.org"
        ]

        for auth_source in authoritative_sources:
            if auth_source in source.lower():
                return 1.0

        # Check if it's from the main book
        if "book_content" in source.lower():
            return 0.9

        # Default score
        return 0.5